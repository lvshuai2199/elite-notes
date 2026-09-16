using System.Collections;


namespace Elite
{
    
    internal class RobotStatePackages
    {
        private List<string> itemTableKey = new List<string>();
        private Dictionary<string, RobotStateItem> itemTable = new Dictionary<string, RobotStateItem>();

        /// <summary>
        /// Add item data to table
        /// </summary>
        /// <param name="item"></param>
        /// <returns></returns>
        internal bool AddItem(RobotStateItem item)
        {
            // If there are items with the same name, append '0' to the end.
            // Typically, these items with the same name are reserved.
            while (itemTable.ContainsKey(item.name))
            {
                item.name += '0';
            }
            itemTable.Add(item.name, item);
            itemTableKey.Add(item.name);
            return true;
        }

        /// <summary>
        /// Get a RobotState item
        /// </summary>
        /// <param name="name">The name of item</param>
        /// <returns>Success is a RobotStateItem, else is null</returns>
        internal RobotStateItem? GetItem(string name)
        {
            if (itemTable.ContainsKey(name))
            {
                return itemTable[name];
            }
            return null;
        }

        /// <summary>
        /// Parse the message.
        /// </summary>
        /// <param name="buffer">message</param>
        /// <exception cref="EliteException.CantFindItem">Maybe configure file not right</exception>
        /// <exception cref="EliteException.ErrorTypeInfo">Unsupported type, requires modification to the source code.</exception>
        /// <exception cref="EliteException.ErrorParsingMessage">Error in parsing the message.</exception>
        internal void ConvertValue(byte[] buffer)
        {
            int count = 0;
            foreach (var name in itemTableKey)
            {
                RobotStateItem? item = null;
                if (!itemTable.TryGetValue(name, out item))
                {
                    throw new EliteException.CantFindItem(name);
                }
                if (item.value is UInt64)
                {
                    Array.Reverse(buffer, count, sizeof(UInt64));
                    item.value = BitConverter.ToUInt64(buffer, count);
                    count += sizeof(UInt64);
                }
                else if (item.value is Int64)
                {
                    Array.Reverse(buffer, count, sizeof(Int64));
                    item.value = BitConverter.ToInt64(buffer, count);
                    count += sizeof(Int64);
                }
                else if (item.value is UInt32)
                {
                    Array.Reverse(buffer, count, sizeof(UInt32));
                    item.value = BitConverter.ToUInt32(buffer, count);
                    count += sizeof(UInt32);
                }
                else if (item.value is Int32)
                {
                    Array.Reverse(buffer, count, sizeof(Int32));
                    item.value = BitConverter.ToInt32(buffer, count);
                    count += sizeof(Int32);
                }
                else if (item.value is UInt16)
                {
                    Array.Reverse(buffer, count, sizeof(UInt16));
                    item.value = BitConverter.ToUInt16(buffer, count);
                    count += sizeof(UInt16);
                }
                else if (item.value is Int16)
                {
                    Array.Reverse(buffer, count, sizeof(Int16));
                    item.value = BitConverter.ToInt16(buffer, count);
                    count += sizeof(Int16);
                }
                else if (item.value is byte)
                {
                    item.value = buffer[count];
                    count++;
                }
                else if (item.value is bool)
                {
                    if (buffer[count] != 0)
                    {
                        item.value = true;
                    }
                    else
                    {
                        item.value = false;
                    }
                    count++;
                }
                else if (item.value is double)
                {
                    Array.Reverse(buffer, count, sizeof(double));
                    item.value = BitConverter.ToDouble(buffer, count);
                    count += sizeof(double);
                }
                else if (item.value is float)
                {
                    Array.Reverse(buffer, count, sizeof(float));
                    item.value = BitConverter.ToSingle(buffer, count);
                    count += sizeof(float);
                }
                else
                {
                    throw new EliteException.ErrorTypeInfo(item.value.GetType());
                }
            }
            if (count != buffer.Length)
            {
                throw new EliteException.ErrorParsingMessage();
            }
        }

    }

    /// <summary>
    /// An entry in RobotState contains the name and value of a piece of data.
    /// </summary>
    internal class RobotStateItem
    {
        internal string name { set; get; }
        internal object value { set; get; }

        public RobotStateItem()
        {
            name = string.Empty;
            value = new object();
        }

        internal RobotStateItem Copy()
        {
            RobotStateItem copy = new RobotStateItem
            {
                name = this.name,
                value = this.value
            };
            return copy;
        }

    }

}
