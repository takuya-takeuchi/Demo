using System;
using System.IO;

namespace ConsoleApplication
{
    public class Program
    {
        public static void Main(string[] args)
        {
            if (args.Length != 1)
            {
                Console.WriteLine("Must specify full path!!");
                return;
            }

            var path = args[0];
            if (!File.Exists(path))
            {
                Console.WriteLine("Specified path does not exist!!");
                return;
            }

            FileStream stream = null;

            try
            {
                stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.None);
                Console.WriteLine($"'{path}' can be opened!!");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Can not open file. Reason: {ex.Message}!!");
            }
            finally
            {
                stream?.Dispose();
            }
        }
    }
}
