using System.Text;
using System.Text.RegularExpressions;


namespace Elite
{
    /// <summary>
    /// Robot runtime exception
    /// </summary>
    public class RobotRuntimeException
    {
        /// <summary>
        /// The line number in the script where the error occurred.
        /// </summary>
        public Int32 scriptLine { get; }

        /// <summary>
        /// The column number in the script where the error occurred.
        /// </summary>
        public Int32 scriptColumn { get; }

        /// <summary>
        /// The error text message.
        /// </summary>
        public string textMessage { get; }

        /// <summary>
        /// Error description
        /// </summary>
        /// <returns>description</returns>
        /// <exception cref="EliteException.NotLoadConfigureFile"></exception>
        public override string ToString()
        {
            if (exception_description.Length <= 0)
            {
                throw new EliteException.NotLoadConfigureFile("RobotRuntimeException");
            }
            return exception_description;
        }

        /// <summary>
        /// Runtime exception text description
        /// </summary>
        private string exception_description = "";

        /// <summary>
        /// Constructor.This constructor just use to when robot throw exception, parse message.(internal use)
        /// </summary>
        /// <param name="total_message">Recv total message</param>
        /// <param name="offset">Exception infomation start byte</param>
        /// <param name="table">Exception description table.See <see cref="RobotRuntimeExceptionTable"></param>
        internal RobotRuntimeException(byte[] total_message, int offset, RobotRuntimeExceptionTable? table) 
        {
            int count = offset;
            Array.Reverse(total_message, count, sizeof(Int32));
            scriptLine = BitConverter.ToInt32(total_message, count);
            count += sizeof(Int32);
            Array.Reverse(total_message, count, sizeof(Int32));
            scriptColumn = BitConverter.ToInt32(total_message, count);
            count += sizeof(Int32);
            textMessage = Encoding.ASCII.GetString(total_message, count, total_message.Length - count);
            if (table != null) 
            {
                exception_description = table.GetDescription(textMessage);
            }
        }
    };

    /// <summary>
    /// Robot error exception
    /// </summary>
    public class RobotErrorException
    {
        /// <summary>
        /// Robot error code
        /// </summary>
        public Int32 code { get; }
        
        /// <summary>
        /// Robot error sub-code
        /// </summary>
        public Int32 subCode { get; }

        /// <summary>
        /// Robot error Level
        /// </summary>
        public enum RobotErrorLevel
        {
            INFO = 0,
            WARNING = 1,
            ERROR = 2,
            SEGMENT_FAULT = 3
        };
        /// <summary>
        /// Robot error level
        /// </summary>
        public RobotErrorLevel level { get; }
        
        /// <summary>
        /// Data about error, like joint index...
        /// </summary>
        public object data { get; }

        /// <summary>
        /// Error code description
        /// </summary>
        /// <return>
        /// Error code description
        /// </return>
        /// <exception cref="EliteException.NotLoadConfigureFile">
        /// If not load error description file, will throw this exception.
        /// </exception>
        public override string ToString()
        {
            if (exception_description.Length <= 0)
            {
                throw new EliteException.NotLoadConfigureFile("RobotErrorException");
            }
            return exception_description;
        }

        /// <summary>
        /// <see cref="data"/> type
        /// </summary>
        /// <remarks>
        /// In the message, 
        /// there's a byte that signals the type of the variable <see cref="data"/> 
        /// and this enumeration type specifies the type associated with the value of that byte.
        /// </remarks>
        private enum RobotErrorDataType
        {
            None = 0,
            Unsigned = 1,
            Signed = 2,
            Float = 3,
            Hex = 4,
            String = 5,
            Joint = 6
        };

        /// <summary>
        /// Robot error exception text description
        /// </summary>
        private string exception_description = "";

        /// <summary>
        /// Constructor.This constructor just use to when robot throw exception, parse message.(internal use)
        /// </summary>
        /// <param name="total_message">Recv total message</param>
        /// <param name="offset">Exception infomation start byte</param>
        /// <param name="table">Exception description table.See <see cref="RobotErrorExceptionTable"></param>
        /// <exception cref="EliteException.ErrorTypeInfo"></exception>
        internal RobotErrorException(byte[] total_message, int offset, RobotErrorExceptionTable? table) 
        {
            int count = offset;
            Array.Reverse(total_message, count, sizeof(Int32));
            code = BitConverter.ToInt32(total_message, count);
            count += sizeof(Int32);

            Array.Reverse(total_message, count, sizeof(Int32));
            subCode = BitConverter.ToInt32(total_message, count);
            count += sizeof(Int32);

            Array.Reverse(total_message, count, sizeof(Int32));
            level = (RobotErrorLevel)BitConverter.ToInt32(total_message, count);
            count += sizeof(Int32);

            Array.Reverse(total_message, count, sizeof(Int32));
            RobotErrorDataType type = (RobotErrorDataType)BitConverter.ToUInt32(total_message, count);
            count += sizeof(Int32);

            switch (type)
            {
                case RobotErrorDataType.None:
                    Array.Reverse(total_message, count, sizeof(UInt32));
                    data = BitConverter.ToUInt32(total_message, count);
                    break;
                case RobotErrorDataType.Unsigned:
                    Array.Reverse(total_message, count, sizeof(UInt32));
                    data = BitConverter.ToUInt32(total_message, count);
                    break;
                case RobotErrorDataType.Signed:
                    Array.Reverse(total_message, count, sizeof(Int32));
                    data = BitConverter.ToInt32(total_message, count);
                    break;
                case RobotErrorDataType.Float:
                    Array.Reverse(total_message, count, sizeof(float));
                    data = BitConverter.ToSingle(total_message, count);
                    break;
                case RobotErrorDataType.Hex:
                    Array.Reverse(total_message, count, sizeof(UInt32));
                    data = BitConverter.ToUInt32(total_message, count);
                    break;
                case RobotErrorDataType.String:
                    data = Encoding.ASCII.GetString(total_message, count, total_message.Length - count);
                    break;
                case RobotErrorDataType.Joint:
                    Array.Reverse(total_message, count, sizeof(UInt32));
                    data = BitConverter.ToUInt32(total_message, count);
                    break;
                default:
                    throw new EliteException.ErrorTypeInfo(type.ToString());
            }
            if (table != null) 
            {
                exception_description = table.GetDescription(code, subCode, data);
            }
        }
    }

    /// <summary>
    /// Robot runtime exception
    /// </summary>
    internal class RobotRuntimeExceptionTable
    {
        /// <summary>
        /// Retrieve the detailed description of the exception based on the runtime exception message.
        /// </summary>
        /// <param name="message">Runtime exception message</param>
        /// <returns>Runtime exception message</returns>
        /// <exception cref="EliteException.CantFindItem">If not load configure or not right file</exception>
        public string GetDescription(string message)
        {
            // The format of received runtime alert messages is 'message:param1:param2:'.
            string[] strings = message.Split(':');
            string? description;
            if (!runtimeExceptionTable.TryGetValue(strings[0], out description))
            {
                throw new EliteException.CantFindItem($"RobotRuntimeException: {strings[0]}");
            }
            // In the runtime exception text file, the text to be replaced is always within '{}'
            string pattern = @"\{(.+?)\}";
            Regex regex = new Regex(pattern);
            // Beacuse format of messages is 'message:param1:param2:',
            // so the index of parameters is start with 1
            int param_count = 1;
            string result = regex.Replace(description, match =>
            {
                string? dataString = strings[param_count];
                param_count++;
                if (dataString != null)
                {
                    return dataString;
                }
                return "";
            });
            return result;
        }

        private Dictionary<string, string> runtimeExceptionTable;
        internal RobotRuntimeExceptionTable(Dictionary<string, string> table) 
        {
            runtimeExceptionTable = table;
        }

    }

    /// <summary>
    /// Robot error excetion table
    /// </summary>
    internal class RobotErrorExceptionTable
    {
        /// <summary>
        /// Constructor. This constructor will save robot exception
        /// </summary>
        /// <param name="table">Robot exception description</param>
        public RobotErrorExceptionTable(Dictionary<int, Dictionary<int, string>> table) 
        {
            errorCodeTable = table;
        }

        /// <summary>
        /// Retrieve the detailed description of the exception based on the error code and error subcode.
        /// </summary>
        /// <param name="code"></param>
        /// <param name="sub_code"></param>
        /// <param name="data"></param>
        /// <returns></returns>
        public string GetDescription(int code, int sub_code, object data) 
        {
            Dictionary<int, string> subCodeTable = errorCodeTable[code];
            string description = subCodeTable[sub_code];
            // In the error code text file, the text to be replaced is always within '{}'
            string pattern = @"\{(.+?)\}";
            Regex regex = new Regex(pattern);
            string result = regex.Replace(description, match =>
            {
                string? dataString = data.ToString();
                if (dataString != null) 
                {
                    return dataString;
                }
                return "";
            });
            return result;
        }
        private Dictionary<int, Dictionary<int, string>> errorCodeTable;
    }
}
