# 一、简介
- 本示例用于解析 Elite CS 系列机器人30001端口的报文，提供了一个 RobotState 类及其使用示例，分别位于 ./RobotState 和 ./ConsoleApp 下。  
- 注意，在使用示例时需要将本目录下的 CS_UserManual_Robot_State_Message.txt 放到可执行程序的同一目录下，或者修改./ConsoleApp中的代码，将本目录下 CS_UserManual_Robot_State_Message.txt 的路径输入给加载配置文件的函数。
- 如果需要更换配置文件，请将英文版的30001协议Excel表格另存为txt格式，并删除有关异常报文描述的部分。


# 二、API说明
## RobotState API 说明
- RobotState(string message_file, string exception_file = "", string runtime_exception_file = "") 
- 描述：构造函数
- 参数：
    - message_file：报文配置文件，即本示例中的`CS_UserManual_Robot_State_Message.txt`
    - exception_file：机器人异常描述文件，即本示例中的`CS_UserManual_ErrorCode.txt`
    - runtime_exception_file：机器人运行时异常描述文件，即本示例中的`CS_UserManual_Runtime_Exception.txt`

- bool Connect(string ip, int port = 30001)
    - 描述：连接机器人
    - 参数：
        - ip：机器人IP
        - port：端口
    - 返回值：成功为true，失败为false。
    

- void Disconnect()
    - 描述：断开链接

- object? GetValue(string name)
    - 描述：获取变量值
    - 参数：
        - name：数据项名称
    - 返回值：成功为变量的值，失败为null。
    - 注：数据项名称由英文版的30001报文说明Excel得到，基本规则为：子报文名称_项目名称。例如，打开表格后能看到“Robot mode sub-package”，即机器人模式子报文，其中数据项“timestamp”在存储在本类中的名字为“Robot_mode_timestamp”

- void SetExceptionHandle(RobotStateExceptionHandler exceptionHandler)
    - 描述：设置当接收到异常报文时的回调函数
    - 参数：
        - exceptionHandler：接收到异常报文时的回调（详细见： RobotStateExceptionHandler）

- delegate void RobotStateExceptionHandler(UInt64 timeStamp, ExceptionSource source, object detail)
    - 描述：异常回调函数
    - 参数：
        - timeStamp：时间戳（毫秒）
        - source：异常源（详细见：ExceptionSource）
        - detail：异常的详细描述（依据异常源不同会分为 RobotRuntimeException 和 RobotErrorException，详见 ExceptionSource 的注）
    
- enum ExceptionSource 
    ```
    enum ExceptionSource
    {
        SAFETY = 99, // 安全控制器
        GUI = 103,  // 示教器UI
        CONTROLLER = 104, // 控制器
        RTSI = 105, // RTSI
        JOINT = 120,    // 关节
        TOOL = 121, //工具
        TP = 122,   // 示教器
        JOINT_FPGA = 200,   // 关节FPGA
        TOOL_FPGA = 201,    // 工具FPGA
        RUN_TIME = 10   // 运行中的程序
    };
    ```
    - 注：当异常源为RUN_TIME是，RobotStateExceptionHandler 的 detail 参数为 RunTimeException 类型，其余为 RobotErrorCode 类型

## RobotRuntimeException 
- Int32 scriptLine
    - 描述：出现异常的程序脚本的行号

- Int32 scriptColumn
    - 描述：出现异常的程序脚本的

- string textMessage
    - 描述：异常的文本描述

- string ToString()
    - 描述：依据加载的配置文件返回异常的详细描述信息
    - 注：如果未加载或者加载了不正确的配置文件，可能会导致异常

## RobotErrorException
- Int32 code
    - 描述：异常码

- Int32 subCode
    - 描述：异常子码

- RobotErrorLevel level
    - 描述：异常等级

- object data
    - 描述：异常的详细数据，可能为Uint32、Int32、float、string

- string ToString()
    - 描述：依据加载的配置文件返回异常的详细描述信息
    - 注：如果未加载或者加载了不正确的配置文件，可能会导致异常

