using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;

namespace ConsoleApplication
{
    public class Program
    {

        internal enum Mode : int
        {
            F_OK = 0,
            X_OK = 1,
            W_OK = 2,
            R_OK = 4
        }

        [DllImport("libc")]
        internal static extern int access(string path, Mode mode);

        public static void Main(string[] args)
        {
            if (args.Length != 1)
            {
                Console.WriteLine("Must specify full path!!");
                return;
            }

            var path = args[0];
            if (access(path, Mode.F_OK) != 0)
            {
                Console.WriteLine("Specified path does not exist!!");
                return;
            }

            Console.WriteLine(access(path, Mode.R_OK) == 0 ? "Can read" : "Can not read");
            Console.WriteLine(access(path, Mode.W_OK) == 0 ? "Can write" : "Can not write");
            Console.WriteLine(access(path, Mode.X_OK) == 0 ? "Can exec" : "Can not exec");
        }
    }

}
