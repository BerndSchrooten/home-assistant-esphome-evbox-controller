#pragma once

#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"

static const char* const TAG = "evbox";

namespace esphome {
namespace evbox {

enum OperatingModes {
  MODE_OFF = 0,
  MODE_MIN = 1,
  MODE_SOLAR = 2,
  MODE_MAX = 3,
  MODE_CUSTOM = 4,
  MODE_ON = 5,
};

class EVBoxDevice : public uart::UARTDevice, public Component {
 public:
  void setup() override;
  void loop() override;
  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }
  void set_fallback_current(float fallback_current) { this->fallback_current_ = fallback_current; }
  void set_min_cc(float min_charge_current) { this->min_charge_current_ = min_charge_current; }
  void set_max_cc(float max_charge_current) { this->max_charge_current_ = max_charge_current; }
  void set_sampletime(float sampletime) { this->sampletime_ = sampletime; }
  void set_samplevalue(float samplevalue) { this->samplevalue_ = samplevalue; }
  void set_samplevalue_sensor(sensor::Sensor *sensor) { this->samplevalue_sensor_ = sensor; }
  void set_calculated_current_sensor(sensor::Sensor *sensor) { this->calculated_current_sensor_ = sensor; }
  void set_custom_charge_current(float custom_charge_current) { this->custom_charge_current_ = custom_charge_current;}

  void set_phase1_current_sensor(sensor::Sensor *sensor) { this->phase1_current_sensor_ = sensor; }
  void set_phase2_current_sensor(sensor::Sensor *sensor) { this->phase2_current_sensor_ = sensor; }
  void set_phase3_current_sensor(sensor::Sensor *sensor) { this->phase3_current_sensor_ = sensor; }
  void set_phase1_voltage_sensor(sensor::Sensor *sensor) { this->phase1_voltage_sensor_ = sensor; }
  void set_phase2_voltage_sensor(sensor::Sensor *sensor) { this->phase2_voltage_sensor_ = sensor; }
  void set_phase3_voltage_sensor(sensor::Sensor *sensor) { this->phase3_voltage_sensor_ = sensor; }
  void set_total_energy_sensor(sensor::Sensor *sensor) { this->total_energy_sensor_ = sensor; }

  void set_setpoint(float setpoint) { this->setpoint_ = setpoint; }
  void set_kp(float kp) { this->kp_ = kp; }
  void set_ki(float ki) { this->ki_ = ki; }
  void set_kd(float kd) { this->kd_ = kd; }

  void set_mode(OperatingModes mode) { this->mode_ = mode; }
  void set_max_power_solar(float power) { this.max_power_solar_ = power; }
 
 protected:
  void send_max_current_( float amp );
  void process_message_( char *msg );
 
  GPIOPin *flow_control_pin_{nullptr};
  sensor::Sensor *samplevalue_sensor_{nullptr};
  sensor::Sensor *calculated_current_sensor_{nullptr}; 
  sensor::Sensor *phase1_current_sensor_{nullptr};
  sensor::Sensor *phase2_current_sensor_{nullptr};
  sensor::Sensor *phase3_current_sensor_{nullptr};
  sensor::Sensor *phase1_voltage_sensor_{nullptr};
  sensor::Sensor *phase2_voltage_sensor_{nullptr};
  sensor::Sensor *phase3_voltage_sensor_{nullptr};

  double total_energy_;
  sensor::Sensor *total_energy_sensor_{nullptr};

  bool receiving_;
  uint8_t received_data_[256];
  uint32_t received_len_;

  float fallback_current_;
  float min_charge_current_;
  float max_charge_current_;
  float output_charge_current_;
  float setpoint_;
  double samplevalue_;
  double sampletime_;
  float custom_charge_current_;
  double kp_;
  double ki_;
  double kd_;

  OperatingModes mode_;
  float max_power_solar_;
};

template<typename... Ts> class SetSampleValueAction : public Action<Ts...> {
    public:
    explicit SetSampleValueAction(EVBoxDevice *parent) : parent_(parent) {}

    TEMPLATABLE_VALUE(float, samplevalue);

    void play(Ts... x) override {
        float samplevalue = this->samplevalue_.value(x...);
        this->parent_->set_samplevalue(samplevalue);
    }
    
    protected:
    EVBoxDevice *parent_;
};

template<typename... Ts> class SetOperatingModeAction : public Action<Ts...> {
    public:
    explicit SetOperatingModeAction(EVBoxDevice *parent) : parent_(parent) {}

    TEMPLATABLE_VALUE(int, mode);

    void play(Ts... x) override {
        OperatingModes mode = (OperatingModes) this->mode_.value(x...);
        this->parent_->set_mode(mode);
    }
    
    protected:
    EVBoxDevice *parent_;
};

template<typename... Ts> class SetCustomChargeCurrentAction : public Action<Ts...> {
    public:
    explicit SetCustomChargeCurrentAction(EVBoxDevice *parent) : parent_(parent) {}

    TEMPLATABLE_VALUE(float, custom_charge_current);

    void play(Ts... x) override {
        float custom_charge_current = this->custom_charge_current_.value(x...);
        this->parent_->set_custom_charge_current(custom_charge_current);
    }
    
    protected:
    EVBoxDevice *parent_;
};

}  // namespace evbox
}  // namespace esphome

