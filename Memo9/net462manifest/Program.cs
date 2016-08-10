using System;
using System.IO;
using System.Linq;

namespace net462manifest
{
    class Program
    {
        static void Main(string[] args)
        {
            var random = new Random();
            var length = 200;
            var str = string.Join("", Enumerable.Range(0, length).Select(s => random.Next(0, 9).ToString()));
            var path = Path.Combine("B:\\", str);
            path = Path.Combine(path, str);

            try
            {
                Directory.CreateDirectory(path);
                Console.WriteLine($"{path} is created.");
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.GetType().Name);
                Console.WriteLine(ex.Message);
            }
        }
    }
}
