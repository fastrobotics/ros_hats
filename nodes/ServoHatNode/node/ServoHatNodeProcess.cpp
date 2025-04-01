#include "ServoHatNodeProcess.h"

namespace ros_hats {
ServoHatNodeProcess::ServoHatNodeProcess() {
}
ServoHatNodeProcess::~ServoHatNodeProcess() {
    delete logger;
}
eros::eros_diagnostic::Diagnostic ServoHatNodeProcess::finish_initialization() {
    eros::eros_diagnostic::Diagnostic diag = get_root_diagnostic();
#ifdef ARCHITECTURE_ARMV7L
    driver = new ServoHatDriver;
#else
    driver = new MockServoHatDriver;
#endif
    driver->init(logger);
    channel_definition_map = driver->get_channel_definitions();
    return diag;
}

void ServoHatNodeProcess::reset() {
}
bool ServoHatNodeProcess::new_servo_command(std::string channel_name, std_msgs::UInt16 msg) {
    auto channel_it = channel_definition_map.find(channel_name);
    if (channel_it == channel_definition_map.end()) {
        logger->log_warn("Got Servo Command for Channel " + channel_name +
                         " but Channel not defined!");
        return false;
    }
    return driver->setServoValue(channel_it->second.pin_number, (int)msg.data);
}
eros::eros_diagnostic::Diagnostic ServoHatNodeProcess::update(double t_dt, double t_ros_time) {
    eros::eros_diagnostic::Diagnostic diag = base_update(t_dt, t_ros_time);
    if (driver->update(t_dt) == false) {
        update_diagnostic(eros::eros_diagnostic::DiagnosticType::ACTUATORS,
                          eros::Level::Type::WARN,
                          eros::eros_diagnostic::Message::DROPPING_PACKETS,
                          "Unable to Update Servo Hat Driver");
    }
    else {
        update_diagnostic(eros::eros_diagnostic::DiagnosticType::ACTUATORS,
                          eros::Level::Type::INFO,
                          eros::eros_diagnostic::Message::NOERROR,
                          "Servo Hat Driver Updated");
    }
    return diag;
}
std::vector<eros::eros_diagnostic::Diagnostic> ServoHatNodeProcess::new_commandmsg(
    eros::command msg) {
    std::vector<eros::eros_diagnostic::Diagnostic> diag_list = base_new_commandmsg(msg);
    if (diag_list.size() == 0) {
        // No currently supported commands.
    }
    else {
        for (auto diag : diag_list) {
            if (diag.level >= eros::Level::Type::INFO) {
                diagnostic_manager.update_diagnostic(diag);
            }
        }
    }
    return diag_list;
}
std::vector<eros::eros_diagnostic::Diagnostic> ServoHatNodeProcess::check_programvariables() {
    std::vector<eros::eros_diagnostic::Diagnostic> diag_list;
    logger->log_warn("No Program Variables Checked.");
    return diag_list;
}
std::string ServoHatNodeProcess::pretty() {
    std::string str = "Node State: " + eros::Node::NodeStateString(get_nodestate());
    str += driver->pretty();
    return str;
}
}  // namespace ros_hats