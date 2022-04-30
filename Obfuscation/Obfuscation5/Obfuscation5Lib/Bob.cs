using System;

namespace Obfuscation5Lib
{
    public class Bob : Lovers,IGreeting
    {
        public void Greeting()
        {
            var time = this.TeachNowTime();
            Console.WriteLine("Hello!! More precisely, It's {0}:{1}:{2}", time.Hour, time.Minute, time.Second);
        }

        public DateTime TeachNowTime()
        {
            return DateTime.Now;
        }
    }
}