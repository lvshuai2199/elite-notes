
namespace Elite
{
    namespace EliteException 
    {
        /// <summary>
        /// Can't find item
        /// </summary>
        public class CantFindItem : Exception
        {
            public CantFindItem(string message) : base(message) { }
        }

        /// <summary>
        /// Unknow type infomation
        /// </summary>
        public class ErrorTypeInfo : Exception
        {
            public ErrorTypeInfo(Type type) : base(type.ToString()) { }
            public ErrorTypeInfo(string type) : base(type) { }
        }

        /// <summary>
        /// Message can't parse or message not right
        /// </summary>
        public class ErrorParsingMessage : Exception
        {
            public ErrorParsingMessage(){ }
        }

        /// <summary>
        /// Configure file is not standard format or configure file corruption.
        /// </summary>
        public class LoadConfigureFileFail : Exception
        {
            public LoadConfigureFileFail(string file) : base(file) { }
        }

        /// <summary>
        /// Must load right configure file then can use function
        /// </summary>
        public class NotLoadConfigureFile : Exception 
        {
            public NotLoadConfigureFile(string file_type) : base(file_type) { }
        }

        /// <summary>
        /// When socket can't receive in normal way
        /// </summary>
        public class  SocketRecvError : Exception
        {
            public SocketRecvError() { }
        }

    }
}
