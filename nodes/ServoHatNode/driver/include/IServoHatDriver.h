/**
 * @file IServoHatDriver.h
 * @author David Gitz
 * @brief
 * @date 2025-02-22
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <eros/Logger.h>

#include <map>
namespace ros_hats {
class IServoHatDriver
{
   public:
    struct ChannelDefinition {
        ChannelDefinition(std::string name, uint8_t pin_number)
            : name(name), pin_number(pin_number) {
        }
        std::string name;
        uint8_t pin_number;
    };
    struct Channel {
        Channel(ChannelDefinition channel_definition) : definition(channel_definition), value(0) {
        }
        ChannelDefinition definition;
        uint16_t value;
    };
    static constexpr int MIN_SERVO_VALUE = 500;
    static constexpr int MEDIUM_SERVO_VALUE = 1500;
    static constexpr int MAX_SERVO_VALUE = 2000;
    struct ServoHatDriverContainer {
        ros::Time timestamp;
    };

    IServoHatDriver() {
    }
    virtual ~IServoHatDriver(){};
    /**
     * @brief Initialize Servo Hat Driver
     *
     * @param logger
     * @return true
     * @return false
     */
    virtual bool init(eros::Logger* logger, int address = 0x40) = 0;

    virtual bool update(double dt) = 0;

    virtual bool setServoValue(int pin_number, int v) = 0;
    /**
     * @brief Finish and Close Driver
     *
     * @return true
     * @return false
     */
    virtual bool finish() = 0;
    virtual std::string pretty(std::string mode = "") = 0;
    virtual std::map<std::string, ChannelDefinition> get_channel_definitions() = 0;
    virtual std::map<uint8_t, Channel> get_channel_map() = 0;
};
}  // namespace ros_hats