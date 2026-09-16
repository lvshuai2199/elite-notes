#ifndef __ROBOTEXCEPTION_HPP__
#define __ROBOTEXCEPTION_HPP__

#include <memory>
#include <any>
#include <string>

class RobotException
{
private:
    
public:
    RobotException();
    ~RobotException();

    static std::shared_ptr<RobotException> unpackException(const uint8_t *buffer);
    
    std::uint64_t timestamp;

    enum ExceptionType : std::uint8_t {
        RUN_TIME_EXCEPTION = 10,
        ERROR_CODE = 6
    };
    ExceptionType exception_type;

    enum Source : std::uint8_t {
        SOURCE_RUNTIME = 10,
        SOURCE_SAFETY = 99,
        SOURCE_GUI = 103,
        SOURCE_CONTROLLER = 104,
        SOURCE_RTSI = 105,
        SOURCE_JOINT = 120,
        SOURCE_TOOL = 121,
        SOURCE_TP = 122,
        SOURCE_JOINT_FPGA = 200,
        SOURCE_TOOL_FPGA = 201
    };
    Source source;

    struct {
        int32_t script_line;
        int32_t script_column;
        std::string runtime_exception_text;
    } runtime_exception;
    
    enum ErrorLevel : std::int32_t {
        INFO = 0,
        WARNING = 1,
        ERROR = 2,
        SEGMENT_FAULT = 3
    };
    
    enum DataType : std::uint32_t {
        DATA_TYPE_NONE = 0,
        DATA_TYPE_UNSIGNED = 1,
        DATA_TYPE_SIGNED = 2,
        DATA_TYPE_FLOAT = 3,
        DATA_TYPE_HEX = 4,
        DATA_TYPE_STRING = 5,
        DATA_TYPE_JOINT = 6
    };
    
    struct {
        int32_t code;
        int32_t subcode;
        ErrorLevel level;
        DataType data_type;
        std::any exception_data;
    } error_exception;

};

#endif
