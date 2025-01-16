#include "bl0940_v2.h"
#include "esphome/core/log.h"
#include <cinttypes>

namespace esphome {
namespace bl0940_v2 {

static const char *const TAG = "bl0940_v2";

static const uint8_t BL0940_V2_READ_COMMAND = 0x58;
static const uint8_t BL0940_V2_FULL_PACKET = 0xAA;
static const uint8_t BL0940_V2_PACKET_HEADER = 0x55;  // 0x58 according to en doc but 0x55 in cn doc

static const uint8_t BL0940_V2_WRITE_COMMAND = 0xA8;
static const uint8_t BL0940_V2_REG_I_FAST_RMS_CTRL = 0x10;
static const uint8_t BL0940_V2_REG_MODE = 0x18;
static const uint8_t BL0940_V2_REG_SOFT_RESET = 0x19;
static const uint8_t BL0940_V2_REG_USR_WRPROT = 0x1A;
static const uint8_t BL0940_V2_REG_TPS_CTRL = 0x1B;

const uint8_t BL0940_V2_INIT[5][6] = {
    // Reset to default
    {BL0940_V2_WRITE_COMMAND, BL0940_V2_REG_SOFT_RESET, 0x5A, 0x5A, 0x5A, 0x38},
    // Enable User Operation Write
    {BL0940_V2_WRITE_COMMAND, BL0940_V2_REG_USR_WRPROT, 0x55, 0x00, 0x00, 0xF0},
    // 0x0100 = CF_UNABLE energy pulse, AC_FREQ_SEL 50Hz, RMS_UPDATE_SEL 800mS
    {BL0940_V2_WRITE_COMMAND, BL0940_V2_REG_MODE, 0x00, 0x10, 0x00, 0x37},
    // 0x47FF = Over-current and leakage alarm on, Automatic temperature measurement, Interval 100mS
    {BL0940_V2_WRITE_COMMAND, BL0940_V2_REG_TPS_CTRL, 0xFF, 0x47, 0x00, 0xFE},
    // 0x181C = Half cycle, Fast RMS threshold 6172
    {BL0940_V2_WRITE_COMMAND, BL0940_V2_REG_I_FAST_RMS_CTRL, 0x1C, 0x18, 0x00, 0x1B}};

void BL0940_V2::loop() {
  DataPacket buffer;
  if (!this->available()) {
    return;
  }
  if (read_array((uint8_t *) &buffer, sizeof(buffer))) {
    if (validate_checksum(&buffer)) {
      received_package(&buffer);
    }
  } else {
    ESP_LOGW(TAG, "Junk on wire. Throwing away partial message");
    while (read() >= 0)
      ;
  }
}

bool BL0940_V2::validate_checksum(DataPacket *data) {
  uint8_t checksum = BL0940_V2_READ_COMMAND;
  // Whole package but checksum
  uint8_t *raw = (uint8_t *) data;
  for (uint32_t i = 0; i < sizeof(*data) - 1; i++) {
    checksum += raw[i];
  }
  checksum ^= 0xFF;
  if (checksum != data->checksum) {
    ESP_LOGW(TAG, "Invalid checksum! 0x%02X != 0x%02X", checksum, data->checksum);
  }
  return checksum == data->checksum;
}

void BL0940_V2::update() {
  this->flush();
  this->write_byte(BL0940_V2_READ_COMMAND);
  this->write_byte(BL0940_V2_FULL_PACKET);
}

void BL0940_V2::setup() {
  // calculate references based on schematic defaults if not set explicitly
  if (!this->voltage_reference_set_) {
    // formula: 79931 / Vref * (R1 * 1000) / (R1 + R2)
    this->voltage_reference_ = 79931 / this->vref_ * (this->r_one_ * 1000) / (this->r_one_ + this->r_two_);
  }
  if (!this->current_reference_set_) {
    // formula: 324004 * RL / Vref
    this->current_reference_ = 324004 * this->r_shunt_ / this->vref_;
  }
  if (!this->power_reference_set_) {
    // formula: 4046 * RL * R1 * 1000 / Vref² / (R1 + R2)
    // this->power_reference_ = 4046 * this->r_shunt_ * this->r_one_ * 1000 / this->vref_ / this->vref_ / (this->r_one_
    // + this->r_two_); or: voltage_reference_ * current_reference_ * 4046 / (324004 * 79931)
    this->power_reference_ = this->voltage_reference_ * this->current_reference_ * 4046 / (324004 * 79931);
  }
  if (!this->energy_reference_set_) {
    // formula: 3600000 * 4046 * RL * R1 * 1000 / (1638.4 * 256) / Vref² / (R1 + R2)
    // or:  power_reference_ * 3600000 / (1638.4 * 256)
    this->energy_reference_ = this->power_reference_ * 3600000 / (1638.4 * 256);
  }

  for (auto *i : BL0940_V2_INIT) {
    this->write_array(i, 6);
    delay(1);
  }
  this->flush();
}

float BL0940_V2::update_temp(sensor::Sensor *sensor, uint16_le_t temperature) const {
  // auto tb = (float) (temperature.h << 8 | temperature.l);
  auto tb = (float) temperature;
  float converted_temp = ((float) 170 / 448) * (tb / 2 - 32) - 45;
  if (sensor != nullptr) {
    if (sensor->has_state() && std::abs(converted_temp - sensor->get_state()) > max_temperature_diff_) {
      ESP_LOGD(TAG, "Invalid temperature change. Sensor: '%s', Old temperature: %f, New temperature: %f",
               sensor->get_name().c_str(), sensor->get_state(), converted_temp);
      return 0.0f;
    }
    sensor->publish_state(converted_temp);
  }
  return converted_temp;
}

void BL0940_V2::received_package(DataPacket *data) {
  // Bad header
  if (data->frame_header != BL0940_V2_PACKET_HEADER) {
    ESP_LOGI(TAG, "Invalid data. Header mismatch: %d", data->frame_header);
    return;
  }

  // cf_cnt is only 24 bits, so track overflows
  uint32_t cf_cnt = (uint24_t) data->cf_cnt;
  cf_cnt |= this->prev_cf_cnt_ & 0xff000000;
  if (cf_cnt < this->prev_cf_cnt_) {
    cf_cnt += 0x1000000;
  }
  this->prev_cf_cnt_ = cf_cnt;

  float v_rms = (uint24_t) data->v_rms / voltage_reference_;
  float i_rms = (uint24_t) data->i_rms / current_reference_;
  float watt = (int24_t) data->watt / power_reference_;
  float total_energy_consumption = cf_cnt / energy_reference_;

  float tps1 = update_temp(internal_temperature_sensor_, data->tps1);
  float tps2 = update_temp(external_temperature_sensor_, data->tps2);

  if (voltage_sensor_ != nullptr) {
    voltage_sensor_->publish_state(v_rms);
  }
  if (current_sensor_ != nullptr) {
    current_sensor_->publish_state(i_rms);
  }
  if (power_sensor_ != nullptr) {
    power_sensor_->publish_state(watt);
  }
  if (energy_sensor_ != nullptr) {
    energy_sensor_->publish_state(total_energy_consumption);
  }

  ESP_LOGV(TAG, "BL0940_V2: U %fV, I %fA, P %fW, Cnt %" PRId32 ", ∫P %fkWh, T1 %f°C, T2 %f°C", v_rms, i_rms, watt,
           cf_cnt, total_energy_consumption, tps1, tps2);
}

void BL0940_V2::dump_config() {  // NOLINT(readability-function-cognitive-complexity)
  ESP_LOGCONFIG(TAG, "BL0940_V2:");
  ESP_LOGCONFIG(TAG, "  Vref: %f", this->vref_);
  ESP_LOGCONFIG(TAG, "  R shunt: %f", this->r_shunt_);
  ESP_LOGCONFIG(TAG, "  R One: %f", this->r_one_);
  ESP_LOGCONFIG(TAG, "  R Two: %f", this->r_two_);
  ESP_LOGCONFIG(TAG, "  Current reference: %f", this->current_reference_);
  ESP_LOGCONFIG(TAG, "  Energy reference: %f", this->energy_reference_);
  ESP_LOGCONFIG(TAG, "  Power reference: %f", this->power_reference_);
  ESP_LOGCONFIG(TAG, "  Voltage reference: %f", this->voltage_reference_);
  LOG_SENSOR("", "Voltage", this->voltage_sensor_);
  LOG_SENSOR("", "Current", this->current_sensor_);
  LOG_SENSOR("", "Power", this->power_sensor_);
  LOG_SENSOR("", "Energy", this->energy_sensor_);
  LOG_SENSOR("", "Internal temperature", this->internal_temperature_sensor_);
  LOG_SENSOR("", "External temperature", this->external_temperature_sensor_);
}

}  // namespace bl0940_v2
}  // namespace esphome