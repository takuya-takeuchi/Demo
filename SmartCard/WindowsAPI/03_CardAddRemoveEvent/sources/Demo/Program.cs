using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Text;
using System.Threading.Tasks;

using Windows.Devices.Enumeration;
using Windows.Devices.SmartCards;

using NLog;

using static System.Array;

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
            var selector = SmartCardReader.GetDeviceSelector();

            var devices = await DeviceInformation.FindAllAsync(selector);
            var firstDevice = devices.FirstOrDefault();
            if (firstDevice == null)
            {
                Logger.Error("No reader connected.");
                return;
            }

            try
            {
                var reader = await SmartCardReader.FromIdAsync(firstDevice.Id);
                reader.CardAdded += OnCardAdded;
                reader.CardRemoved += OnCardRemoved;
            }
            catch (Exception ex)
            {
                Logger.Info(ex.Message);
            }

            Logger.Info("Press any key to exit.");
            Console.ReadKey();
        }

        #region Events

        private static void OnCardRemoved(SmartCardReader sender, CardRemovedEventArgs args)
        {
            Logger.Info($"CardRemoved: {args.SmartCard.Reader.Name}");
        }

        private static void OnCardAdded(SmartCardReader sender, CardAddedEventArgs args)
        {
            Logger.Info($"CardAdded: {args.SmartCard.Reader.Name}");
        }

        #endregion

        #endregion

    }

}
