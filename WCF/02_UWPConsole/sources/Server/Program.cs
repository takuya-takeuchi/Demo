using System;
using System.ServiceModel;
using System.ServiceModel.Description;

using NLog;

using Server.Services;

namespace Server
{

    internal sealed class Program
    {

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Methods

        private static void Main(string[] args)
        {
            var baseAddress = args[0];
            Logger.Info($"Start on {baseAddress}");
            
            // Create the ServiceHost.
            using (var host = new ServiceHost(typeof(GreetingService), new Uri(baseAddress)))
            {
                // Enable metadata publishing.
                var smb = new ServiceMetadataBehavior();
                smb.HttpGetEnabled = true;
                smb.MetadataExporter.PolicyVersion = PolicyVersion.Policy15;
                host.Description.Behaviors.Add(smb);
                host.Open();

                Logger.Info($"The service is ready at {0}", baseAddress);
                Logger.Info("Press <Enter> to stop the service.");
                Console.ReadLine();

                host.Close();
            }
        }

        #endregion

    }

}
