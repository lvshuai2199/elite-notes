using System.Text.RegularExpressions;

namespace Elite
{
    /// <summary>
    /// Load file and get message format
    /// </summary>
    internal class RobotStateLoadFile
    {
        /// <summary>
        /// Load file and analyze
        /// </summary>
        /// <param name="path"></param>
        /// <returns><see cref="RobotStatePackages">Message </returns>
        internal static RobotStatePackages? LoadFile(string path) 
        {
            RobotStatePackages result = new RobotStatePackages();
            try 
            {
                using (StreamReader sr = new StreamReader(path))
                {
                    string pattern = @"sub-package\([0-9]+ bytes\)";
                    Regex regex = new Regex(pattern);
                    string? line;
                    bool isForeach = false;
                    string prefix = "";
                    List<RobotStateItem> foreachItemList = new List<RobotStateItem>();
                    // Read the file content line by line.
                    while ((line = sr.ReadLine()) != null)
                    {
                        MatchCollection matches = regex.Matches(line);
                        if (matches.Count > 0) 
                        {
                            prefix = "";
                            int pos = line.IndexOf(matches[0].Value);
                            if (pos != -1) 
                            { 
                                prefix = line.Substring(0, pos);
                                prefix = prefix.Replace(' ', '_');
                            }
                        }
                        if (line.Contains("foreach joint:")) 
                        { 
                            isForeach = true;
                        }
                        if (line.Contains("\tend")) 
                        {
                            isForeach = false;
                            for (int i = 0; i < 6; i++) 
                            {
                                foreach (RobotStateItem it in foreachItemList)
                                {
                                    RobotStateItem temp = it.Copy();
                                    temp.name += i.ToString();
                                    if (!result.AddItem(temp))
                                    {
                                        return null;
                                    }
                                }
                            }
                            foreachItemList.Clear();
                        }

                        RobotStateItem? item = AnalyzeItem(line, prefix);
                        if (item != null) 
                        {
                            if (isForeach)
                            {
                                foreachItemList.Add(item);
                            }
                            else 
                            {
                                if (!result.AddItem(item))
                                {
                                    return null;
                                }
                            }
                        }
                    }
                }
            }
            catch (FileNotFoundException)
            {
                return null;
            }
            catch (IOException)
            {
                return null;
            }

            return result;
        }

        /// <summary>
        /// In the message configuration file, the variable type names and their corresponding types.
        /// </summary>
        static Dictionary<string, object> dataTypeList = new Dictionary<string, object>()
        {
            {"uint64_t", new UInt64() }, 
            {"int64_t", new Int64() }, 
            {"uint32_t", new UInt32() }, 
            {"int32_t", new Int32() },
            {"uint16_t", new UInt16() },
            {"int16_t", new Int16() },
            {"uint8_t", new byte() },
            {"int8_t", new byte() },
            {"bool", new bool() },
            {"double", new double() },
            {"float", new float() }
        };
        /// <summary>
        /// Parse one line of the message configuration file and return the parsing result.
        /// </summary>
        /// <param name="line">Line of the message configuration file</param>
        /// <param name="prefix">The added prefix typically represents the type of this portion of the message.</param>
        /// <returns><see cref="RobotStateItem"/>Parsing result</returns>
        static RobotStateItem? AnalyzeItem(string line, string prefix) 
        {
            foreach (var item in dataTypeList) 
            {
                int pos = line.IndexOf(item.Key);
                if (pos == -1) 
                {
                    continue;
                }
                string name = "";
                int nameStartPos = pos + item.Key.Length + 1;
                for (int i = nameStartPos; line[i] != '\0'; i++) 
                { 
                    if (line[i] == ' ' || line[i] == '\t') 
                    {
                        break;
                    }
                    name += line[i];
                }
                RobotStateItem result = new RobotStateItem();
                result.name = prefix + name;
                result.value = item.Value;
                return result;
            }
            return null;
        }

    }


}
