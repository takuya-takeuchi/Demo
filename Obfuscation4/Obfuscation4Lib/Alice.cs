using System;

namespace Obfuscation4Lib
{
    internal class Alice : Lovers, IGreeting
    {
        public void Greeting()
        {
            Console.WriteLine("Hello!! It's already {0} o'clock.", this.GetNowHour());
        }

        private int GetNowHour()
        {
            return DateTime.Now.Hour;
        }
    }
}