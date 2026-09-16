
namespace Elite
{
    /// <summary>
    /// Provide a configuration file for parsing exception information.
    /// </summary>
    internal class RobotExceptionLoadFile
    {
        /// <summary>
        /// Load robot error exception file
        /// </summary>
        /// <param name="path">File path</param>
        /// <returns><see cref="RobotErrorExceptionTable"/></returns>
        internal static RobotErrorExceptionTable? LoadErrorExceptionFile(string path) 
        {
            try
            {
                using (StreamReader sr = new StreamReader(path, System.Text.Encoding.UTF8)) 
                {
                    Dictionary<int, Dictionary<int, string>>? exceptDict = RobotErrorExceptionFileAnalyze(sr);
                    if (exceptDict == null || exceptDict.Count <= 0) 
                    {
                        return null;
                    }
                    return new RobotErrorExceptionTable(exceptDict);
                }
            }
            catch (Exception)
            {
                throw;
            }
        }

        /// <summary>
        /// Load robot runtime exception file
        /// </summary>
        /// <param name="path"></param>
        /// <returns></returns>
        internal static RobotRuntimeExceptionTable? LoadRuntimeExceptionFile(string path)
        {
            try
            {
                using (StreamReader sr = new StreamReader(path, System.Text.Encoding.UTF8))
                {
                    Dictionary<string, string>? exceptDict = RobotRuntimeExceptionFileAnalyze(sr);
                    if (exceptDict == null || exceptDict.Count <= 0)
                    {
                        return null;
                    }
                    return new RobotRuntimeExceptionTable(exceptDict);
                }
            }
            catch (Exception)
            {
                throw;
            }
        }

        /// <summary>
        /// Analyze the error code description file and generate an error code dictionary.
        /// </summary>
        /// <param name="sr">File stream</param>
        /// <returns> Error code dictionary </returns>
        private static Dictionary<int, Dictionary<int, string>>? RobotErrorExceptionFileAnalyze(StreamReader sr) 
        {
            Dictionary<int, Dictionary<int, string>> result = new Dictionary<int, Dictionary<int, string>>();
            Dictionary<int, string>? subcodeDict = null;
            // Skip the first line of the file since it's the header.
            string? line = sr.ReadLine();
            int errorCode = -1;
            int subcode = -1;
            string? description = null;
            // Read the file content line by line.
            while ((line = sr.ReadLine()) != null)
            {
                string[] strings = line.Split(new char[] { '\t' }, StringSplitOptions.RemoveEmptyEntries);
                // When the error code changes,
                // there are 3 columns in 1 row,
                // and a change in the error code represents a new exception type.
                if (strings.Length == 3)
                {
                    if (errorCode >= 0 && (subcodeDict != null))
                    {
                        result.Add(errorCode, subcodeDict);
                    }
                    errorCode = int.Parse(strings[0]);
                    subcode = int.Parse(strings[1]);
                    description = strings[2];
                    subcodeDict = new Dictionary<int, string>();
                    subcodeDict?.Add(subcode, description);
                }
                else if (strings.Length == 2)
                {
                    subcode = int.Parse(strings[0]);
                    description = strings[1];
                    subcodeDict?.Add(subcode, description);
                }
            }
            // The last subcodeDict don't add in result
            if (!result.ContainsKey(errorCode)) 
            {
                if (subcodeDict == null)
                {
                    return null;
                }
                result.Add(errorCode, subcodeDict);
            }
            return result;
        }

        /// <summary>
        /// Analyze the runtime exception description file and generate an dictionary.
        /// </summary>
        /// <param name="sr"></param>
        /// <returns></returns>
        private static Dictionary<string, string>? RobotRuntimeExceptionFileAnalyze(StreamReader sr) 
        {
            string? line;
            Dictionary<string, string> result = new Dictionary<string, string>();
            while ((line = sr.ReadLine()) != null) 
            {
                string[] strings = line.Split(new char[] { '\t' }, StringSplitOptions.RemoveEmptyEntries);
                if (strings.Length == 2) 
                {
                    result.Add(strings[0], strings[1]);
                }
            }
            if (result.Count <= 0)
            {
                return null;
            }
            return result;
        }
    }
}
