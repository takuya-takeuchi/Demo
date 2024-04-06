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
                Logger.Info($"SmartCardReader: {firstDevice.Name}");
                var smartCardReader = await SmartCardReader.FromIdAsync(firstDevice.Id);

                Logger.Info("Press any key When put card on device");
                Console.ReadKey();
                var cards = await smartCardReader.FindAllCardsAsync();

                var smartCard = cards.FirstOrDefault();
                if (smartCard == null)
                {
                    Logger.Error("Failed to get any card");
                    return;
                }

                using var smartCardConnection = await smartCard.ConnectAsync();

                var result = await ConnectToJapanesePublicKeyInfrastructure(smartCardConnection);
                var success = IsSuccess(result);
                Logger.Info($"\tSELECT FILE JPKI: {success} [{BitConverter.ToString(result)}]");
                if (!success)
                {
                    Logger.Error("No JPKI file");
                    return;
                }

                result = await ConnectToAuthenticationPIN(smartCardConnection);
                success = IsSuccess(result);
                Logger.Info($"\tSELECT FILE PIN: {success} [{BitConverter.ToString(result)}]");
                if (!success)
                {
                    Logger.Error("No PIN file");
                    return;
                }

                result = await GetAuthenticationPINRetryCount(smartCardConnection);
                success = TryGetRetryCount(result, out var count);
                if (!success)
                {
                    Logger.Error("Failed to get retry count");
                    return;
                }
                Logger.Info($"\tRETRY COUNT: {count}");
            }
            catch (Exception ex)
            {
                Logger.Info(ex.Message);
            }

            Logger.Info("Press any key to exit.");
            Console.ReadKey();
        }

        #region Helpers

        private static bool IsSuccess(IReadOnlyList<byte> response)
        {
            var sw1 = response[0];
            var sw2 = response[1];
            return sw1 == 0x90 && sw2 == 0x00;
        }

        private static async Task<byte[]> CommandApdu(SmartCardConnection smartCardConnection, byte[] apdu)
        {
            var response = await smartCardConnection.TransmitAsync(apdu.AsBuffer());
            return response.ToArray();
        }

        private static async Task<byte[]> CommandApdu(SmartCardConnection smartCardConnection, byte cla, byte ins, byte p1, byte p2)
        {
            var apdu = new[] { cla, ins, p1, p2 };
            return await CommandApdu(smartCardConnection, apdu);
        }

        private static async Task<byte[]> ConnectToAuthenticationPIN(SmartCardConnection smartCardConnection)
        {
            var apdu = new byte[] { 0x00, 0xA4, 0x02, 0x0C, 0x02, 0x00, 0x18 };
            return await CommandApdu(smartCardConnection, apdu);
        }

        private static async Task<byte[]> ConnectToJapanesePublicKeyInfrastructure(SmartCardConnection smartCardConnection)
        {
            var apdu = new byte[] { 0x00, 0xA4, 0x04, 0x0C, 0x0A, 0xD3, 0x92, 0xF0, 0x00, 0x26, 0x01, 0x00, 0x00, 0x00, 0x01 };
            return await CommandApdu(smartCardConnection, apdu);
        }

        private static async Task<byte[]> GetAuthenticationPINRetryCount(SmartCardConnection smartCardConnection)
        {
            var apdu = new byte[] { 0x00, 0x20, 0x00, 0x80 };
            return await CommandApdu(smartCardConnection, apdu);
        }

        private static bool TryGetRetryCount(IReadOnlyList<byte> response, out int count)
        {
            count = 0;
            
            var sw1 = response[0];
            var sw2 = response[1];
            if (sw1 != 0x63)
                return false;

            count = sw2 & 0x0F;
            return true;
        }

        #endregion

        #endregion

    }

}
