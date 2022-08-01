#include "evbox.h"
#include "current_controller.h"

namespace esphome
{
  namespace evbox
  {

    CurrentController *current_controller;
    const char hex[] = "0123456789ABCDEF";

    void EVBoxDevice::setup()
    {
      if (this->flow_control_pin_ != nullptr)
      {
        // Set flowcontrolpin
        this->flow_control_pin_->setup();
        // Flow control to RX
        this->flow_control_pin_->digital_write(0);
      }
      ESP_LOGD(TAG, "Setup");

      this->received_len_ = 0;
      this->total_energy_ = 0;
      this->receiving_ = false;
      this->calculated_current_ = 0;
      this->real_max_phase_current_ = 0;

      current_controller = new CurrentController(&(this->real_max_phase_current_), &(this->calculated_current_), &(this->target_current_), this->kp_);
      current_controller->set_output_limits(this->min_charge_current_, this->max_charge_current_);
    }

    void EVBoxDevice::loop()
    {
      const uint32_t now = millis();
      static uint32_t lastSample = now;
      static uint32_t lastMsg = 0;
      static OperatingModes lastMode = MODE_OFF;

      if (lastMode != this->mode_)
      {
        lastMode = this->mode_;
        switch (this->mode_)
        {
        case MODE_OFF:
          this->target_current_ = 0;
          current_controller->set_output_limits(0, 0);
          this->calculated_current_ = 0;
          break;
        case MODE_MIN:
          this->target_current_ = this->min_charge_current_;
          this->calculated_current_ = this->min_charge_current_;
          break;
        case MODE_SOLAR:
          this->target_current_ = this->target_solar_power_consumption_ / this->voltage_ / this->nb_active_phases_;
          break;
        case MODE_MAX:
          this->target_current_ = this->max_charge_current_;
          this->calculated_current_ = this->max_charge_current_;
          break;
        case MODE_CUSTOM:
          this->target_current_ = this->custom_charge_current_;
          this->calculated_current_ = this->custom_charge_current_;
        }
      }

      if (this->mode_ == MODE_CUSTOM && this->target_current_ != this->custom_charge_current_)
      {
        this->target_current_ = this->custom_charge_current_;
      }

      if (this->mode_ == MODE_SOLAR)
      {
        this->target_current_ = this->target_solar_power_consumption_ / this->voltage_ / this->nb_active_phases_;
      }

      // Capture EVBox data
      if (this->available())
      {
        uint8_t c;
        this->read_byte(&c);

        if (c == 2)
        { // Message Start
          receiving_ = true;
          received_len_ = 0;
        }
        else if (c == 3 && received_len_ > 8)
        { // Message End
          receiving_ = false;
          received_data_[received_len_] = 0;
          process_message_((char *)received_data_);
          received_len_ = 0;
        }
        else if (receiving_ && c >= 48 && c <= 70)
        { // Capture message data
          if (received_len_ < 255)
            received_data_[received_len_++] = c;
        }
        else
        { // Invalid data
          received_len_ = 0;
        }
      }

      // Take a sample if time has passed
      if ((now - lastSample) > 1000.0 * (this->sampletime_))
      {

        if (this->target_solar_power_consumption_)
          this->target_solar_power_consumption_sensor_->publish_state(this->target_solar_power_consumption_);

        if (this->mode_ == MODE_OFF)
          this->calculated_current_ = 0.0;
        else
        {
          if (!std::isnan(this->real_max_phase_current_))
            current_controller->compute();
        }

        lastSample = now;

        if (!std::isnan(this->calculated_current_))
        {
          send_max_current_(this->calculated_current_);
          this->real_max_phase_current_ = this->calculated_current_;
        }

        if (this->calculated_current_sensor_)
          this->calculated_current_sensor_->publish_state(this->calculated_current_);

        if (this->target_current_sensor_)
          this->target_current_sensor_->publish_state(this->target_current_);
      }
    }

    void EVBoxDevice::process_message_(char *msg)
    {
      int i;
      int msglen = strlen(msg);

      // ESP_LOGD(TAG, "RX: %s", msg );

      // Check length and header
      if (msglen == 56 && strncmp(msg, "A08069", 6) == 0)
      {
        int cm = 0;
        int cx = 0;

        ESP_LOGD(TAG, "Processing received message");

        for (i = 0; i < 52; i++)
        {
          cm = (cm + msg[i]) & 255;
          cx = cx ^ (msg[i]);
        }

        uint8_t csm = (uint8_t)cm;
        uint8_t csx = (uint8_t)cx;

        // Checksum validation
        if (hex[csm >> 4] == msg[52] && hex[csm & 15] == msg[53] &&
            hex[csx >> 4] == msg[54] && hex[csx & 15] == msg[55])
        {
          // Read MID metered values from EVSE
          char meter[9];
          strncpy(meter, &msg[44], 8);
          meter[8] = 0;

          this->total_energy_ = strtoul(meter, NULL, 16);
          if (total_energy_sensor_ && this->total_energy_ != 0)
            total_energy_sensor_->publish_state(this->total_energy_);

          char current[5];
          current[4] = 0;

          strncpy(current, &msg[20], 4);
          this->real_phase1_current_ = 0.1 * strtoul(current, NULL, 16);
          if (phase1_current_sensor_)
            phase1_current_sensor_->publish_state(this->real_phase1_current_);

          strncpy(current, &msg[24], 4);
          this->real_phase2_current_ = 0.1 * strtoul(current, NULL, 16);
          if (phase2_current_sensor_)
            phase2_current_sensor_->publish_state(this->real_phase2_current_);

          strncpy(current, &msg[28], 4);
          this->real_phase3_current_ = 0.1 * strtoul(current, NULL, 16);
          if (phase3_current_sensor_)
            phase3_current_sensor_->publish_state(this->real_phase3_current_);

        }
      }
    }

    void EVBoxDevice::send_max_current_(float amp)
    {
      // MaxChargingCurrent command, timeout=60s, current after timeout 6A
      char buf[35] = "80A06900__00__00__003C003C003C003C";
      int ta = round(10 * amp);

      // Set current values (fill in the blanks)
      buf[8] = buf[12] = buf[16] = hex[(ta >> 4) & 15];
      buf[9] = buf[13] = buf[17] = hex[(ta >> 0) & 15];

      // Calc checksum
      int cs;

      cs = 0;
      for (int i = 0; buf[i]; i++)
        cs = (cs + buf[i]) & 255;
      uint8_t csm = (uint8_t)cs;

      cs = 0;
      for (int i = 0; buf[i]; i++)
        cs = cs ^ (buf[i]);
      uint8_t csx = (uint8_t)cs;

      ESP_LOGD(TAG, "Send charge current");

      if (this->flow_control_pin_ != nullptr)
        this->flow_control_pin_->digital_write(1); // TX mode

      // StartOfMessage
      this->write_byte(2);
      // Actual Message
      this->write_array((uint8_t *)buf, 34);
      // Add checksum to message
      this->write_byte(hex[csm >> 4]);
      this->write_byte(hex[csm & 15]);
      this->write_byte(hex[csx >> 4]);
      this->write_byte(hex[csx & 15]);
      // EndOfMessage
      this->write_byte(3);
      this->flush();

      if (this->flow_control_pin_ != nullptr)
        this->flow_control_pin_->digital_write(0); // RX mode
    }

  } // namespace evbox
} // namespace esphome