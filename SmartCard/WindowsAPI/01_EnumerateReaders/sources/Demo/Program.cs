using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Windows.Devices.Enumeration;
using Windows.Devices.SmartCards;

using NLog;

namespace Demo
{

    internal sealed class Program
    {

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Methods

        private static async Task Main()
        {
            string selector = SmartCardReader.GetDeviceSelector();

            var smartCardReaders = await DeviceInformation.FindAllAsync(selector);
            foreach (var reader in smartCardReaders)
                Logger.Info($"Connected device: {reader.Name}");

            Logger.Info("Press any key to exit.");
            Console.ReadKey();
        }

        #endregion

    }

}
