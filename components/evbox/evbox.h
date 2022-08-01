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

static const char *const TAG = "evbox";

namespace esphome
{
  namespace evbox
  {

    enum OperatingModes
    {
      MODE_OFF = 0,
      MODE_MIN = 1,
      MODE_SOLAR = 2,
      MODE_MAX = 3,
      MODE_CUSTOM = 4,
    };

    class EVBoxDevice : public uart::UARTDevice, public Component
    {
    public:
      void setup() override;
      void loop() override;
      void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }
      void set_fallback_current(float fallback_current) { this->fallback_current_ = fallback_current; }
      void set_min_cc(float min_charge_current) { this->min_charge_current_ = min_charge_current; }
      void set_max_cc(float max_charge_current) { this->max_charge_current_ = max_charge_current; }
      void set_voltage(float voltage) { this->voltage_ = voltage; }
      void set_nb_active_phases(int active_phases) { this->nb_active_phases_ = active_phases;}
      void set_sampletime(float sampletime) { this->sampletime_ = sampletime; }
      void set_calculated_current_sensor(sensor::Sensor *sensor) { this->calculated_current_sensor_ = sensor; }
      void set_custom_charge_current(float custom_charge_current) { this->custom_charge_current_ = custom_charge_current; }
      void set_target_solar_power_consumption(float target_solar_power_consumption) { this->target_solar_power_consumption_ = target_solar_power_consumption; }
      void set_target_solar_power_consumption_sensor(sensor::Sensor *sensor) { this->target_solar_power_consumption_sensor_ = sensor; }

      void set_target_current_sensor(sensor::Sensor *sensor) { this->target_current_sensor_ = sensor; }
      void set_phase1_current_sensor(sensor::Sensor *sensor) { this->phase1_current_sensor_ = sensor; }
      void set_phase2_current_sensor(sensor::Sensor *sensor) { this->phase2_current_sensor_ = sensor; }
      void set_phase3_current_sensor(sensor::Sensor *sensor) { this->phase3_current_sensor_ = sensor; }
      void set_total_energy_sensor(sensor::Sensor *sensor) { this->total_energy_sensor_ = sensor; }

      void set_mode(OperatingModes mode) { this->mode_ = mode; }

      void set_kp(float kp) { this->kp_ = kp; }
      void set_ki(float ki) { this->ki_ = ki; }
      void set_kd(float kd) { this->kd_ = kd; }

    protected:
      void send_max_current_(float amp);
      // void send_command_(char *buffer);
      void process_message_(char *msg);
      // char *int_to_hex_byte_array_(int number, int nb_characters);

      GPIOPin *flow_control_pin_{nullptr};
      sensor::Sensor *target_solar_power_consumption_sensor_{nullptr};
      sensor::Sensor *target_current_sensor_{nullptr};
      sensor::Sensor *calculated_current_sensor_{nullptr};
      sensor::Sensor *phase1_current_sensor_{nullptr};
      sensor::Sensor *phase2_current_sensor_{nullptr};
      sensor::Sensor *phase3_current_sensor_{nullptr};
      sensor::Sensor *total_energy_sensor_{nullptr};
      bool receiving_;
      uint8_t received_data_[256];
      uint32_t received_len_;

      float fallback_current_;
      float min_charge_current_;
      float max_charge_current_;
      float voltage_;
      int nb_active_phases_;
      double custom_charge_current_;
      double sampletime_;
      OperatingModes mode_;

      double real_phase1_current_;
      double real_phase2_current_;
      double real_phase3_current_;
      double real_max_phase_current_;
      double total_energy_;
      
      double calculated_current_;
      
      double target_current_;
      double target_solar_power_consumption_;
      
      double kp_;
      double ki_;
      double kd_;
    };

    template <typename... Ts>
    class SetTargetSolarPowerConsumptionAction : public Action<Ts...>
    {
    public:
      explicit SetTargetSolarPowerConsumptionAction(EVBoxDevice *parent) : parent_(parent) {}

      TEMPLATABLE_VALUE(float, target_solar_power_consumption);

      void play(Ts... x) override
      {
        float target_solar_power_consumption = this->target_solar_power_consumption_.value(x...);
        this->parent_->set_target_solar_power_consumption(target_solar_power_consumption);
      }

    protected:
      EVBoxDevice *parent_;
    };

    template <typename... Ts>
    class SetOperatingModeAction : public Action<Ts...>
    {
    public:
      explicit SetOperatingModeAction(EVBoxDevice *parent) : parent_(parent) {}

      TEMPLATABLE_VALUE(int, mode);

      void play(Ts... x) override
      {
        OperatingModes mode = (OperatingModes)this->mode_.value(x...);
        this->parent_->set_mode(mode);
      }

    protected:
      EVBoxDevice *parent_;
    };

    template <typename... Ts>
    class SetCustomChargeCurrentAction : public Action<Ts...>
    {
    public:
      explicit SetCustomChargeCurrentAction(EVBoxDevice *parent) : parent_(parent) {}

      TEMPLATABLE_VALUE(float, custom_charge_current);

      void play(Ts... x) override
      {
        float custom_charge_current = this->custom_charge_current_.value(x...);
        this->parent_->set_custom_charge_current(custom_charge_current);
      }

    protected:
      EVBoxDevice *parent_;
    };

  } // namespace evbox
} // namespace esphome
