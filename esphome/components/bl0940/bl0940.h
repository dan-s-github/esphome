#pragma once

#include "esphome/core/component.h"
#include "esphome/core/datatypes.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace bl0940 {

// static const float BL0940_UREF = 33000;
// static const float BL0940_IREF = 275000;  // 2750 from tasmota. Seems to generate values 100 times too high

// Values according to BL0940 application note:
// https://www.belling.com.cn/media/file_object/bel_product/BL0940/guide/BL0940_APPNote_TSSOP14_V1.04_EN.pdf

static const float BL0940_VREF = 1.218;  // Vref = 1.218
static const float BL0940_RL = 1;        // RL = 1 mΩ
static const float BL0940_R1 = 0.51;     // R1 = 0.51 kΩ
static const float BL0940_R2 = 1950;     // R2 = 5 x 390 kΩ -> 1950 kΩ

static const float BL0940_UREF = 17158.9201389365;  // should be: 79931 / Vref * (R1 * 1000) / (R1 + R2)
static const float BL0940_IREF = 266013.136288998;  // should be: 324004 * RL / Vref
static const float BL0940_PREF = 713.104696500825;  // should be: 4046 * RL * R1 * 1000 / Vref² / (R1 + R2)
static const float BL0940_EREF = 6120.6267056536;   // should be: 3600000 / (1638.4 * 256) * BL0940_PREF

// Measured to 297J  per click according to power consumption of 5 minutes
// Converted to kWh (3.6MJ per kwH). Used to be 256 * 1638.4
// static const float BL0940_EREF = 3.6e6 / 297;

// Caveat: All these values are big endian (low - middle - high)
struct DataPacket {
  uint8_t frame_header;    // value of 0x58 according to docs. 0x55 according to Tasmota real world tests. Reality wins.
  uint24_le_t i_fast_rms;  // 0x00
  uint24_le_t i_rms;       // 0x04
  uint24_t RESERVED0;      // reserved
  uint24_le_t v_rms;       // 0x06
  uint24_t RESERVED1;      // reserved
  int24_le_t watt;         // 0x08
  uint24_t RESERVED2;      // reserved
  uint24_le_t cf_cnt;      // 0x0A
  uint24_t RESERVED3;      // reserved
  uint16_le_t tps1;        // 0x0c
  uint8_t RESERVED4;       // value of 0x00
  uint16_le_t tps2;        // 0x0c
  uint8_t RESERVED5;       // value of 0x00
  uint8_t checksum;        // checksum
} __attribute__((packed));

class BL0940 : public PollingComponent, public uart::UARTDevice {
 public:
  void set_voltage_sensor(sensor::Sensor *voltage_sensor) { voltage_sensor_ = voltage_sensor; }
  void set_current_sensor(sensor::Sensor *current_sensor) { current_sensor_ = current_sensor; }
  void set_power_sensor(sensor::Sensor *power_sensor) { power_sensor_ = power_sensor; }
  void set_energy_sensor(sensor::Sensor *energy_sensor) { energy_sensor_ = energy_sensor; }

  void set_internal_temperature_sensor(sensor::Sensor *internal_temperature_sensor) {
    internal_temperature_sensor_ = internal_temperature_sensor;
  }
  void set_external_temperature_sensor(sensor::Sensor *external_temperature_sensor) {
    external_temperature_sensor_ = external_temperature_sensor;
  }

  void set_reference_voltage(float vref) { this->vref_ = vref; }
  void set_resistor_shunt(float resistor_shunt) { this->r_shunt_ = resistor_shunt; }
  void set_resistor_one(float resistor_one) { this->r_one_ = resistor_one; }
  void set_resistor_two(float resistor_two) { this->r_two_ = resistor_two; }

  void set_current_reference(float current_ref) {
    this->current_reference_ = current_ref;
    this->current_reference_set_ = true;
  }
  void set_energy_reference(float energy_ref) {
    this->energy_reference_ = energy_ref;
    this->energy_reference_set_ = true;
  }
  void set_power_reference(float power_ref) {
    this->power_reference_ = power_ref;
    this->power_reference_set_ = true;
  }
  void set_voltage_reference(float voltage_ref) {
    this->voltage_reference_ = voltage_ref;
    this->voltage_reference_set_ = true;
  }

  void loop() override;

  void update() override;
  void setup() override;
  void dump_config() override;

 protected:
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  // NB This may be negative as the circuits is seemingly able to measure
  // power in both directions
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *energy_sensor_{nullptr};
  sensor::Sensor *internal_temperature_sensor_{nullptr};
  sensor::Sensor *external_temperature_sensor_{nullptr};

  // Max difference between two measurements of the temperature. Used to avoid noise.
  float max_temperature_diff_{0};

  float vref_ = BL0940_VREF;
  float r_shunt_ = BL0940_RL;
  float r_one_ = BL0940_R1;
  float r_two_ = BL0940_R2;

  // Divide by this to turn into Watt
  float power_reference_ = BL0940_PREF;
  bool power_reference_set_ = false;
  // Divide by this to turn into Volt
  float voltage_reference_ = BL0940_UREF;
  bool voltage_reference_set_ = false;
  // Divide by this to turn into Ampere
  float current_reference_ = BL0940_IREF;
  bool current_reference_set_ = false;
  // Divide by this to turn into kWh
  float energy_reference_ = BL0940_EREF;
  bool energy_reference_set_ = false;

  float update_temp_(sensor::Sensor *sensor, uint16_le_t packed_temperature) const;

  uint32_t prev_cf_cnt_ = 0;

  bool validate_checksum(DataPacket *data);
  void received_package_(DataPacket *data);
};
}  // namespace bl0940
}  // namespace esphome
