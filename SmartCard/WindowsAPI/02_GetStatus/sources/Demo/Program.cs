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
            if (devices.Count == 0)
            {
                Logger.Error("No reader connected.");
                return;
            }

            foreach (var device in devices)
            {
                try
                {
                    var reader = await SmartCardReader.FromIdAsync(device.Id);
                    var cards = await reader.FindAllCardsAsync();

                    Logger.Info($"Connected device: {device.Name}");
                    if (cards.Count == 0)
                    {
                        Logger.Info("No card");
                    }
                    else
                    {
                        foreach (var card in cards)
                        {
                            var cardStatus = await card.GetStatusAsync();
                            Logger.Info($"\tStatus: {cardStatus}");
                            var atr = await card.GetAnswerToResetAsync();
                            Logger.Info($"\tATR: {BitConverter.ToString(atr?.ToArray() ?? Empty<byte>())}");
                        }
                    }
                }
                catch (Exception ex)
                {
                    Logger.Info(ex.Message);
                }
            }

            Logger.Info("Press any key to exit.");
            Console.ReadKey();
        }

        #endregion

    }

}
