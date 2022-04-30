using System;
using System.Runtime.InteropServices;

namespace CsharpApp
{
    class Program
    {
        [DllImport("libCppLib.so")]
        private static extern int Add(int x, int y);

        static void Main(string[] args)
        {
            var x = int.Parse(args[0]);
            var y = int.Parse(args[1]);
            var sum = Add(x, y);
            Console.WriteLine($"{x} + {y} = {sum}");
        }
    }
}
