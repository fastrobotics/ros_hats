/**
 * @file BaseServoHatDriver.h
 * @author David Gitz
 * @brief
 * @date 2025-02-23
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <eros_utility/ConvertUtility.h>
#include <eros_utility/PrettyUtility.h>

#include "IServoHatDriver.h"
namespace ros_hats {

class BaseServoHatDriver : public IServoHatDriver
{
   public:
    BaseServoHatDriver() {
    }
    virtual ~BaseServoHatDriver() {
    }
    std::map<std::string, ChannelDefinition> get_channel_definitions() override {
        return channel_definition_map;
    }
    std::map<uint8_t, Channel> get_channel_map() {
        return channel_map;
    }
    bool update(double dt) override {
        run_time += dt;
        return true;
    }
    bool setServoValue(int pin_number, int v) {
        auto channel_it = channel_map.find(pin_number);
        if (channel_it == channel_map.end()) {
            logger->log_warn("Pin " + std::to_string(pin_number) + " Not Defined!");
            return false;
        }
        if ((v < MIN_SERVO_VALUE) || (v > MAX_SERVO_VALUE)) {
            return false;
        }
        return true;
    }
    std::string pretty(std::string mode) {
        std::string str;
        if (mode == "") {
            str = "\nRuntime: " + std::to_string(run_time) + " (sec).\n";
            if (channel_map.size() == 0) {
                str += "--- Channels: 0---\n";
            }
            else {
                str += "--- Channels ---\n";
                for (auto channel : channel_map) {
                    str += "\t[" + std::to_string(channel.second.definition.pin_number) +
                           "] Name: " + channel.second.definition.name +
                           " V: " + std::to_string(channel.second.value) + "\n";
                }
            }
        }
        else if (mode == "simple") {
            str += " ";
            if (channel_map.size() == 0) {
                str = " No Channels Defined.";
            }
            else {
                for (auto channel : channel_map) {
                    str += "P: " + std::to_string(channel.second.definition.pin_number) +
                           " V: " + std::to_string(channel.second.value) + "\n";
                }
            }
        }
        return str;
    }

   protected:
    eros::Logger* logger;
    std::map<std::string, ChannelDefinition> channel_definition_map;
    std::map<uint8_t, Channel> channel_map;

   private:
    double run_time{0.0};
};
}  // namespace ros_hats