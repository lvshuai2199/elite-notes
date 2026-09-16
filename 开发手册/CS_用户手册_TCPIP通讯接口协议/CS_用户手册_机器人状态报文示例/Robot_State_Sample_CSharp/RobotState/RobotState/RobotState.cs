using System.Net.Sockets;


namespace Elite 
{
    public class RobotState
    {
        /// <summary>
        /// Robot exception source
        /// </summary>
        public enum ExceptionSource
        {
            /// <summary>
            /// Security controller
            /// </summary>
            SAFETY = 99,

            /// <summary>
            /// Teaching UI (User Interface)
            /// </summary>
            GUI = 103,

            /// <summary>
            /// Controller
            /// </summary>
            CONTROLLER = 104,

            /// <summary>
            /// RTSI protocol
            /// </summary>
            RTSI = 105,

            /// <summary>
            /// Joint
            /// </summary>
            JOINT = 120,

            /// <summary>
            /// Tool
            /// </summary>
            TOOL = 121,

            /// <summary>
            /// Teaching pendant
            /// </summary>
            TP = 122,

            /// <summary>
            /// Joint FPGA
            /// </summary>
            JOINT_FPGA = 200,

            /// <summary>
            /// Tool FPGA
            /// </summary>
            TOOL_FPGA = 201
        };

        /// <summary>
        /// Robot exception type
        /// </summary>
        public enum ExceptionType
        {
            /// <summary>
            /// The robot encounters an exception while running a command
            /// </summary>
            RUNTIME_EXCEPTION = 10,

            /// <summary>
            /// The robot error
            /// </summary>
            ERROR_CODE = 6
        };

        /// <summary>
        /// When robot throw an exception, will call the function.
        /// </summary>
        /// <param name="timeStamp">exception time stamp</param>
        /// <param name="source">exception source</param>
        /// <param name="detail"> exception instance, the type is <see cref="RobotErrorException"/>
        ///  and <see cref="RobotRuntimeException"/>
        /// </param>
        public delegate void RobotStateExceptionHandler(UInt64 timeStamp, ExceptionSource source, ExceptionType type, object detail);

        /// <summary>
        /// RobotState constructor
        /// </summary>
        /// <param name="message_file">Robot state message format file</param>
        /// <param name="exception_file">Robot exception description file</param>
        /// <param name="runtime_exception_file">Robot runtime exception description file</param>
        /// <exception cref="EliteException.LoadConfigureFileFail">
        /// This exception will be raised if the file format is incorrect.
        /// </exception>
        public RobotState(string message_file, string exception_file = "", string runtime_exception_file = "") 
        {
            configPackages = RobotStateLoadFile.LoadFile(message_file);
            if (configPackages == null)
            {
                throw new EliteException.LoadConfigureFileFail(message_file);
            }
            if (exception_file.Length > 0) 
            {
                robotErrorExceptionTable = RobotExceptionLoadFile.LoadErrorExceptionFile(exception_file);
                if (robotErrorExceptionTable == null)
                {
                    throw new EliteException.LoadConfigureFileFail(exception_file);
                }
            }
            if (runtime_exception_file.Length > 0)
            {
                robotRuntimeExceptionTable = RobotExceptionLoadFile.LoadRuntimeExceptionFile(runtime_exception_file);
                if (robotRuntimeExceptionTable == null) 
                {
                    throw new EliteException.LoadConfigureFileFail(runtime_exception_file);
                }
            }
        }

        /// <summary>
        /// Connect to robot
        /// </summary>
        /// <param name="ip"></param>
        /// <param name="port"></param>
        /// <returns>true if success</returns>
        public bool Connect(string ip, int port = 30001)
        {
            if (socket != null) 
            {
                Disconnect();
            }
            socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            socket.Connect(ip, port);
            socket.NoDelay = true;

            if (socket.Connected)
            {
                isReciveThreadAlive = true;
                recvThread = new Thread(RecvThread);
                recvThread.Start();
            }

            return socket.Connected;
        }

        /// <summary>
        /// Disconnect
        /// </summary>
        public void Disconnect()
        {
            isReciveThreadAlive = false;
            recvThread?.Join();
            if (socket != null)
            {
                socket.Disconnect(false);
                socket = null;
            }
        }

        /// <summary>
        /// Get the value associated with the variable name.
        /// </summary>
        /// <param name="name"></param>
        /// <returns>Value of name</returns>
        public object? GetValue(string name)
        {
            lock (configPackagesLock)
            {
                RobotStateItem? item = configPackages?.GetItem(name);
                return item?.value;
            }
        }

        /// <summary>
        /// Set a callback function for robot exceptions
        /// </summary>
        /// <param name="exceptionHandler"><see cref="RobotStateExceptionHandler"></param>
        public void SetExceptionHandle(RobotStateExceptionHandler exceptionHandler)
        {
            lock (this)
            {
                robotStateExceptionHandler = exceptionHandler;
            }
        }

        /// <summary>
        /// Send message to robot
        /// </summary>
        /// <param name="message"></param>
        /// <returns></returns>
        public bool SendToRobot(byte[] message) 
        {
            if (socket != null) 
            {
                if (socket.Send(message) != message.Length) 
                {
                    return false;
                }
                return true;
            }
            return false;
        }


        private Socket? socket = null;
        private int socketReciveTimeout = 5000;
        private Thread? recvThread = null;
        private bool isReciveThreadAlive = false;
        
        // The variable will be used in more than two threads, so configPackagesLock is a thread mutex.
        private RobotStatePackages? configPackages = null;
        private readonly object configPackagesLock = new object();
        
        // When robot throw an exception, if user set, will call this function 
        private RobotStateExceptionHandler? robotStateExceptionHandler = null;
        // The 30001 message head's bytes
        private const int PACKAGE_HEAD_LEN = 5;
        // A table save robot exception description
        private RobotErrorExceptionTable? robotErrorExceptionTable = null;
        // A table save robot runtime exception description
        private RobotRuntimeExceptionTable? robotRuntimeExceptionTable = null;
        private void RecvThread()
        {
            while (isReciveThreadAlive)
            {
                UpdateRobotState();
            }
        }

        private enum MessageType : byte
        {
            MESSAGE_TYPE_ROBOT_STATE = 16,
            MESSAGE_TYPE_ROBOT_MESSAGE = 20,
        }
        private void UpdateRobotState()
        {
            while (isReciveThreadAlive)
            {
                byte[] head = new byte[PACKAGE_HEAD_LEN];
                if (SocketRecv(head) != head.Length)
                {
                    return;
                }
                // The first 4 bytes of the message represent the message length.
                Array.Reverse(head, 0, 4);
                UInt32 totalMsgLen = BitConverter.ToUInt32(head, 0);
                // This situation is considered an error in the message.
                if (totalMsgLen <= 0 || head[4] == 0)
                {
                    return;
                }
                byte[] buffer = new byte[totalMsgLen];
                // Restore the byte order of the message header and copy it to the buffer,
                // as it will be needed for further parsing.
                Array.Reverse(head, 0, 4);
                Array.Copy(head, 0, buffer, 0, head.Length);
                // Receiving the remaining messages.
                // The offset is PACKAGE_HEAD_LEN,
                // because the first "PACKAGE_HEAD_LEN" bytes in buffer already contain the message header data.
                if (SocketRecv(buffer, PACKAGE_HEAD_LEN) != (buffer.Length - PACKAGE_HEAD_LEN))
                {
                    throw new EliteException.SocketRecvError();
                }
                AnalyzeMessage(buffer,(MessageType)head[4]);
            }
        }


        private void AnalyzeMessage(byte[] buffer, MessageType type)
        {
            switch (type)
            {
                case MessageType.MESSAGE_TYPE_ROBOT_STATE:
                    AnalyzeRobotState(buffer);
                    break;
                case MessageType.MESSAGE_TYPE_ROBOT_MESSAGE:
                    AnalyzeRobotMessage(buffer);
                    break;
            }
        }

        private void AnalyzeRobotState(byte[] buffer)
        {
            if (configPackages == null)
            {
                return;
            }
            lock (configPackagesLock)
            {
                configPackages.ConvertValue(buffer);
            }

        }

        private void AnalyzeRobotMessage(byte[] buffer) 
        {
            // The message header has already been parsed, so there's no need to parse it again.
            int count = PACKAGE_HEAD_LEN;
            UInt64 timeStamp = BitConverter.ToUInt64(buffer, count);
            count += sizeof(UInt64);
            ExceptionSource exceptionSource = (ExceptionSource)buffer[count];
            count++;
            ExceptionType exceptionType = (ExceptionType)buffer[count];
            count++;
            if (exceptionType == ExceptionType.RUNTIME_EXCEPTION)
            {
                if (robotStateExceptionHandler != null) 
                {
                    RobotRuntimeException re = new RobotRuntimeException(buffer, count, robotRuntimeExceptionTable);
                    robotStateExceptionHandler(timeStamp, exceptionSource, exceptionType, re);
                }
            }
            else if (exceptionType == ExceptionType.ERROR_CODE)
            {
                if (robotStateExceptionHandler != null)
                {
                    RobotErrorException ee = new RobotErrorException(buffer, count, robotErrorExceptionTable);
                    robotStateExceptionHandler(timeStamp, exceptionSource, exceptionType, ee);
                }
            }
        }

        private int SocketRecv(byte[] buffer, int offset= 0)
        {
            if (socket == null) 
            {
                return -1;
            }
            try
            {
                int totalBytesReceived = 0;
                int needRecvBytes = buffer.Length - offset;
                while (totalBytesReceived < needRecvBytes)
                {
                    socket.ReceiveTimeout = socketReciveTimeout;
                    int bytesRead = socket.Receive(buffer, totalBytesReceived + offset, needRecvBytes - totalBytesReceived, SocketFlags.None);
                    if (bytesRead == 0)
                    {
                        break;
                    }
                    totalBytesReceived += bytesRead;
                }
                return totalBytesReceived;
            }
            catch (SocketException)
            { 
                socket.Disconnect(false);
                return -1;
            }
        }

    }

}