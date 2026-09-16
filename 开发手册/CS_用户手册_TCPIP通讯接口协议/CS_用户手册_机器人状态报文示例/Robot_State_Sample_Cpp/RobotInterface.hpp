#ifndef __ROBOT_INTERFACE_HPP__
#define __ROBOT_INTERFACE_HPP__

#include "RobotState.hpp"

#include <stdint.h>
#include <string>
#include <vector>


class RobotInterface : public RobotState {
   public:
    uint64_t getTimeStamp();
    bool isRobotPowerOn();
    bool isEmergencyStopped();
    bool isRobotProtectiveStopped();
    bool isProgramRunning();
    bool isProgramPaused();

    enum class RobotMode : uint8_t {
        ROBOT_MODE_DISCONNECTED = 0,
        ROBOT_MODE_CONFIRM_SAFETY = 1,
        ROBOT_MODE_BOOTING = 2,
        ROBOT_MODE_POWER_OFF = 3,
        ROBOT_MODE_POWER_ON = 4,
        ROBOT_MODE_IDLE = 5,
        ROBOT_MODE_BACKDRIVE = 6,
        ROBOT_MODE_RUNNING = 7,
        ROBOT_MODE_UPDATING_FIRMWARE = 8,
        ROBOT_MODE_WAITING_CALIBRATION = 9
    };
    RobotMode getRobotMode();

    enum class RobotControlMode : uint8_t {
        CONTROL_MODE_POSITION = 0,
        CONTROL_MODE_TORQUE = 1

    };
    RobotControlMode getRobotControlMode();
    double getTargetSpeedFraction();
    double getSpeedScaling();
    double getTargetSpeedFractionLimit();

    enum class RobotSpeedMode : uint8_t {
        UNRESTRICTED = 0,
        MANUAL_HIGH_SPEED = 1,
        MANUAL_REDUCED_SPEED = 2

    };
    RobotSpeedMode getRobotSpeedMode();

    bool isRobotSystemInAlarm();
    bool isInPackageMode();

    std::vector<double> getJointActualPos();
    std::vector<double> getJointTargetPos();
    std::vector<double> getJointActualVelocity();
    std::vector<int32_t> getJointTargetPluse();
    std::vector<int32_t> getJointActualPluse();
    std::vector<int32_t> getJointZeroPluse();
    std::vector<float> getJointCurrent();
    std::vector<float> getJointVoltage();
    std::vector<float> getJointTemperature();
    std::vector<float> getJointTorques();
    enum class JointMode : uint8_t {
        MODE_RESET = 235,
        MODE_SHUTTING_DOWN = 236,
        MODE_BACKDRIVE = 238,
        MODE_POWER_OFF = 239,
        MODE_READY_FOR_POWEROFF = 240,
        MODE_NOT_RESPONDING = 245,
        MODE_MOTOR_INITIALISATION = 246,
        MODE_BOOTING = 247,
        MODE_BOOTLOADER = 249,
        MODE_VIOLATION = 251,
        MODE_FAULT = 252,
        MODE_RUNNING = 253,
        MODE_IDLE = 255

    };
    std::vector<JointMode> getJointMode();

    std::vector<double> getTcpPosition();
    std::vector<double> getTcpOffset();

    std::vector<double> getLimitMinJoint();
    std::vector<double> getLimitMaxJoint();
    std::vector<double> getMaxVelocityJoint();
    std::vector<double> getMaxAccJoint();
    double getDefaultVelocityJoint();
    double getDefaultAccJoint();
    double getDefaultToolVelocity();
    double getDefaultToolAcc();
    double getEqRadius();
    std::vector<double> getDhAJoint();
    std::vector<double> getDhDJoint();
    std::vector<double> getDhAlphaJoint();
    uint32_t getBoardVersion();
    uint32_t getControlBoxType();

    enum class RobotType : uint32_t {
        ROBOT_TYPE_6203 = 6203,
        ROBOT_TYPE_6206 = 6206,
        ROBOT_TYPE_6212 = 6212

    };
    RobotType getRobotType();

    enum class RobotStruct : uint32_t {
        ROBOT_STRUCTURE_TYPE_60 = 60,
        ROBOT_STRUCTURE_TYPE_62 = 62,
        ROBOT_STRUCTURE_TYPE_70 = 70

    };
    RobotStruct getRobotStruct();

    std::vector<bool> getStandardDigitalInput();
    std::vector<bool> getStandardDigitalOutput();
    std::vector<bool> getConfigureDigitalInput();
    std::vector<bool> getConfigureDigitalOutput();
    std::vector<bool> getToolDigitalInput();
    std::vector<bool> getToolDigitalOutput();
    enum class AnalogMode : uint8_t { CURRENT = 0, VOLTAGE = 1 };
    std::vector<AnalogMode> getStandardAnalogOutputMode();
    std::vector<AnalogMode> getStandardAnalogInputMode();
    AnalogMode getToolAnalogOutputMode();
    AnalogMode getToolAnalogInputMode();
    std::vector<double> getStandardAnalogOutputValue();
    std::vector<double> getStandardAnalogInputValue();
    double getToolAnalogOutputValue();
    double getToolAnalogInputValue();
    float getBoardTemperature();
    float getRobotVoltage();
    float getRobotCurrent();
    float getIOCurrent();
    enum class SafetyMode : uint8_t {
        SAFETY_MODE_NORMAL = 1,
        SAFETY_MODE_REDUCED = 2,
        SAFETY_MODE_PROTECTIVE_STOP = 3,
        SAFETY_MODE_RECOVERY = 4,
        SAFETY_MODE_SAFEGUARD_STOP = 5,
        SAFETY_MODE_SYSTEM_EMERGENCY_STOP = 6,
        SAFETY_MODE_ROBOT_EMERGENCY_STOP = 7,
        SAFETY_MODE_VIOLATION = 8,
        SAFETY_MODE_FAULT = 9,
        SAFETY_MODE_VALIDATE_JOINT_ID = 10,
        SAFETY_MODE_UNDEFINED_SAFETY_MODE = 11,
        SAFETY_MODE_AUTOMATIC_MODE_SAFEGUARD_STOP = 12,
        SAFETY_MODE_SYSTEM_THREE_POSITION_ENABLING_STOP = 13
    };
    SafetyMode getBordSafeMode();
    bool isRobotInReducedMode();
    bool getOperationalModeSelectorInput();
    bool getThreepositionEnablingDeviceInput();
    SafetyMode getMasterboardSafetyMode();

    bool isFreedriveButtonPressed();
    bool isFreedriveIOEnabled();
    bool isDynamicCollisionDetectEnabled();

    float getToolVoltage();
    float getToolCurrent();
    enum class ToolOutputVoltage : uint8_t { VOLTAGE_0_LEVEL = 0, VOLTAGE_12_LEVEL = 12, VOLTAGE_24_LEVEL = 24 };
    ToolOutputVoltage getToolOutputVoltage();
    float getToolTemperature();
    typedef JointMode ToolMode;
    ToolMode getToolMode();

    uint32_t getSafetyCRCNum();
    enum class SafeOptMode : int8_t { NONE = -1, AUTOMATIC = 0, MANUAL = 1 };
    SafeOptMode getSafetyOperationalMode();
    std::vector<double> getCurrentElbowPos();
    double getElbowRadius();

    bool isToolRS485Enable();
    uint32_t getToolRS485Baudrate();
    uint32_t getToolRS485Parity();
    uint32_t getToolRS485Stopbits();
    bool isToolRS485ModbusMode();
    enum class ToolRS485Usage : uint8_t { SCRIPT_MODE, DAEMON_MODE };
    ToolRS485Usage getToolRS485Usage();

    template <typename T>
    std::vector<T> getStatesVector(const char* state_list[], int list_size) {
        std::vector<T> result;
        for (int i = 0; i < list_size; i++) {
            result.push_back(getItemData<T>(state_list[i], nullptr));
        }
        return result;
    }

    RobotInterface();
    ~RobotInterface();
};

#endif