#pragma once

#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/mqtt/custom_mqtt_device.h"

static const char* const TAG = "evbox";

namespace esphome {
namespace mqtt {
namespace evbox {

class EVBoxDevice : public uart::UARTDevice, public mqtt::CustomMQTTDevice, public Component {
 public:
  void setup() override;
  void loop() override;
  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }
  void set_min_cc(float min_charge_current) { this->min_charge_current_ = min_charge_current; }
  void set_max_cc(float max_charge_current) { this->max_charge_current_ = max_charge_current; }
  void set_setpoint(float setpoint) { this->setpoint_ = setpoint; }
  void set_kp(float kp) { this->kp_ = kp; }
  void set_ki(float ki) { this->ki_ = ki; }
  void set_kd(float kd) { this->kd_ = kd; }
  
 protected:
  void on_mqtt_receive_(const std::string& topic, const std::string& payload);
  GPIOPin *flow_control_pin_{nullptr};
  uint8_t receive_data_[256];
  uint32_t received_len_;
  float min_charge_current_;
  float max_charge_current_;
  float setpoint_;
  float kp_;
  float ki_;
  float kd_;
  
  bool receiving_;
};

}  // namespace evbox
}  // namespace mqtt
}  // namespace esphome
