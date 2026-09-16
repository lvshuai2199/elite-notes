using Elite;
using static Elite.RobotState;

namespace ProgramSpace
{

    class Program
    {
        public static void RobotExceptionCallback(UInt64 timeStamp, ExceptionSource source, ExceptionType type, object detail) 
        {
            Console.WriteLine($"Time {timeStamp} Exception source {source.ToString()}");
            if (type == ExceptionType.RUNTIME_EXCEPTION) 
            {
                RobotRuntimeException runTimeException = (RobotRuntimeException)detail;
                Console.WriteLine($"\t Script line {runTimeException.scriptLine} ");
                Console.WriteLine($"\t Script Column {runTimeException.scriptColumn} ");
                Console.WriteLine($"\t Detail {runTimeException.textMessage} ");
                Console.WriteLine($"\t Description {runTimeException.ToString()} ");
            }
            else 
            { 
                RobotErrorException robotErrorCode = (RobotErrorException)detail;
                Console.WriteLine($"\t Code {robotErrorCode.code}");
                Console.WriteLine($"\t Sub Code {robotErrorCode.subCode}");
                Console.WriteLine($"\t Level {robotErrorCode.level.ToString()}");
                Console.WriteLine($"\t Data {robotErrorCode.data}");
                Console.WriteLine($"\t Description {robotErrorCode.ToString()} ");
            }
        }

        public static int Main() 
        {
            RobotState rs = new RobotState("CS_UserManual_Robot_State_Message.txt", "CS_UserManual_ErrorCode.txt", "CS_UserManual_Runtime_Exception.txt");
            
            if (!rs.Connect("172.26.38.30")) 
            {
                return -1;
            }
            rs.SetExceptionHandle(RobotExceptionCallback);
            while (true) 
            { 
                Console.WriteLine($"time stamp {rs.GetValue("Robot_mode_timestamp")}");
                Thread.Sleep(500);
            }
        }
    }

}
