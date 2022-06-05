using System;
using Sentry;

namespace Demo
{

    internal class Program
    {

        private static void Main(string[] args)
        {
            using (SentrySdk.Init(o =>
                   {
                       o.Dsn = "http://7faf7bce38604a47a2a25ff456572836@localhost:9000/2";
                       // When configuring for the first time, to see what the SDK is doing:
                       o.Debug = true;
                       // Set traces_sample_rate to 1.0 to capture 100% of transactions for performance monitoring.
                       // We recommend adjusting this value in production.
                       o.TracesSampleRate = 1.0;
                   }))
            {
                // App code goes here. Dispose the SDK before exiting to flush events.
                var left = int.Parse(args[0]);
                var right = int.Parse(args[1]);
                var divided = left / right;
                Console.WriteLine(divided);
            }
        }
        
    }
    
}
