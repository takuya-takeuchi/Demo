using System;

namespace ConsoleApplication
{
    public class Program
    {
        public static void Main(string[] args)
        {
            Console.WriteLine($"Argument is '{args[0]}'");

            var path = System.IO.Path.GetFullPath(args[0]);
            Console.WriteLine($"Path is '{path}'");

            foreach(var f in new System.IO.DirectoryInfo(path).GetFiles())
                Console.WriteLine($"{f.Name}");
        }
    }
}
