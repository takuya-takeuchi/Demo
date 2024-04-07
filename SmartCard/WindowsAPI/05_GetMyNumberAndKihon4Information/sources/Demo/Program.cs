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

                var result = await SelectFileKenmenNyuryokuHojo(smartCardConnection);
                var success = IsSuccess(result);
                Logger.Info($"\tSELECT FILE 券面入力補助AP: {success} [{BitConverter.ToString(result)}]");
                if (!success)
                {
                    Logger.Error("No 券面入力補助AP");
                    return;
                }

                result = await SelectFileKenmenNyuryokuHojoPIN(smartCardConnection);
                success = IsSuccess(result);
                Logger.Info($"\tSELECT FILE 券面入力補助用PIN: {success} [{BitConverter.ToString(result)}]");
                if (!success)
                {
                    Logger.Error("No 券面入力補助用PIN");
                    return;
                }

                byte[] pin;
                while (true)
                {
                    Logger.Info("Press enter 認証用PIN (4 digit)");
                    var input = ReadLineWithMask();
                    pin = Encoding.ASCII.GetBytes(input);
                    if (pin.Length != 4)
                    {
                        Logger.Error("認証用PIN shall be 4 digit");
                        continue;
                    }

                    break;
                }

                result = await VerifyKenmenNyuryokuHojoPIN(smartCardConnection, pin);
                Logger.Info($"\tVERIFY: [{BitConverter.ToString(result)}]");
                success = IsSuccess(result);
                if (!success)
                {
                    Logger.Info("\tNG");
                    success = TryGetRetryCount(result, out var count);
                    if (!success)
                    {
                        Logger.Error("Failed to get retry count");
                        return;
                    }

                    Logger.Info($"\tRETRY COUNT: {count}");
                    return;
                }

                Logger.Info("\tOK");

                result = await SelectFileMyNumber(smartCardConnection);
                success = IsSuccess(result);
                Logger.Info($"\tSELECT FILE マイナンバー: {success} [{BitConverter.ToString(result)}]");
                if (!success)
                {
                    Logger.Error("No マイナンバー");
                    return;
                }

                result = await ReadBinaryMyNumber(smartCardConnection);
                Logger.Info($"\tREAD BINARY マイナンバー: {success} [{BitConverter.ToString(result)}]");
                if (!success)
                {
                    Logger.Error("No マイナンバー");
                    return;
                }

                success = TryGetMyNumber(result, out var myNumber);
                if (!success)
                {
                    Logger.Error("Failed to get 基本4情報");
                    return;
                }

                Logger.Info($"\t\tマイナンバー: {myNumber}");

                result = await SelectFileKihon4Information(smartCardConnection);
                success = IsSuccess(result);
                Logger.Info($"\tSELECT FILE 基本4情報: {success} [{BitConverter.ToString(result)}]");
                if (!success)
                {
                    Logger.Error("No 基本4情報");
                    return;
                }

                result = await ReadBinaryKihon4Information(smartCardConnection);
                success = TryGetBinaryLength(result, out var length);
                Logger.Info($"\tREAD BINARY 基本4情報 (長さ): {success} [{BitConverter.ToString(result)}]");
                if (!success)
                {
                    Logger.Error("No 基本4情報 (長さ)");
                    return;
                }

                result = await ReadBinaryKihon4InformationHeader(smartCardConnection);
                Logger.Info($"\tREAD BINARY 基本4情報 ヘッダー: {success} [{BitConverter.ToString(result)}]");
                if (!success)
                {
                    Logger.Error("No 基本4情報 ヘッダー");
                    return;
                }

                success = TryGetKihon4InformationHeader(result, out var contentLength);
                if (!success)
                {
                    Logger.Error("Failed to get 基本4情報 ヘッダー");
                    return;
                }

                result = await ReadBinaryKihon4Information(smartCardConnection, contentLength);
                Logger.Info($"\tREAD BINARY 基本4情報: {success} [{BitConverter.ToString(result)}]");
                if (!success)
                {
                    Logger.Error("No 基本4情報");
                    return;
                }

                success = TryGetKihon4Information(result, out var name, out var address, out var dateOfBirth, out var sex);
                if (!success)
                {
                    Logger.Error("Failed to get 基本4情報");
                    return;
                }

                Logger.Info($"\t\t名前: {name}");
                Logger.Info($"\t\t住所: {address}");
                Logger.Info($"\t\t生年月日: {dateOfBirth}");
                Logger.Info($"\t\t性別: {sex}");
            }
            catch (Exception ex)
            {
                Logger.Info(ex.Message);
            }

            Logger.Info("Press any key to exit.");
            Console.ReadKey();
        }

        #region Helpers

        private static string ReadLineWithMask()
        {
            string input = "";
            ConsoleKeyInfo key;

            do
            {
                key = Console.ReadKey(true);
                if (key.Key != ConsoleKey.Backspace && key.Key != ConsoleKey.Enter)
                {
                    input += key.KeyChar;
                    Console.Write("*");
                }
                else
                {
                    if (key.Key == ConsoleKey.Backspace && input.Length > 0)
                    {
                        input = input.Substring(0, (input.Length - 1));
                        Console.Write("\b \b");
                    }
                }
            }
            while (key.Key != ConsoleKey.Enter);

            Console.Write(Environment.NewLine);

            return input;
        }

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

        private static async Task<byte[]> SelectFileKenmenNyuryokuHojo(SmartCardConnection smartCardConnection)
        {
            var apdu = new byte[] { 0x00, 0xA4, 0x04, 0x0C, 0x0A, 0xD3, 0x92, 0x10, 0x00, 0x31, 0x00, 0x01, 0x01, 0x04, 0x08 };
            return await CommandApdu(smartCardConnection, apdu);
        }

        private static async Task<byte[]> SelectFileKenmenNyuryokuHojoPIN(SmartCardConnection smartCardConnection)
        {
            var apdu = new byte[] { 0x00, 0xA4, 0x02, 0x0C, 0x02, 0x00, 0x11 };
            return await CommandApdu(smartCardConnection, apdu);
        }

        private static async Task<byte[]> SelectFileKihon4Information(SmartCardConnection smartCardConnection)
        {
            var apdu = new byte[] { 0x00, 0xA4, 0x02, 0x0C, 0x02, 0x00, 0x02 };
            return await CommandApdu(smartCardConnection, apdu);
        }

        private static async Task<byte[]> SelectFileMyNumber(SmartCardConnection smartCardConnection)
        {
            var apdu = new byte[] { 0x00, 0xA4, 0x02, 0x0C, 0x02, 0x00, 0x01 };
            return await CommandApdu(smartCardConnection, apdu);
        }

        private static async Task<byte[]> ReadBinaryKihon4Information(SmartCardConnection smartCardConnection)
        {
            var apdu = new byte[] { 0x00, 0xB0, 0x00, 0x02, 0x01 };
            return await CommandApdu(smartCardConnection, apdu);
        }

        private static async Task<byte[]> ReadBinaryKihon4Information(SmartCardConnection smartCardConnection, byte length)
        {
            var apdu = new byte[] { 0x00, 0xB0, 0x00, 0x00, length };
            return await CommandApdu(smartCardConnection, apdu);
        }

        private static async Task<byte[]> ReadBinaryKihon4InformationHeader(SmartCardConnection smartCardConnection)
        {
            var apdu = new byte[] { 0x00, 0xB0, 0x00, 0x00, 0x05 };
            return await CommandApdu(smartCardConnection, apdu);
        }

        private static async Task<byte[]> ReadBinaryMyNumber(SmartCardConnection smartCardConnection)
        {
            var apdu = new byte[] { 0x00, 0xB0, 0x00, 0x00, 0x00 };
            return await CommandApdu(smartCardConnection, apdu);
        }

        private static async Task<byte[]> VerifyKenmenNyuryokuHojoPIN(SmartCardConnection smartCardConnection, byte[] pin)
        {
            var apdu = new byte[] { 0x00, 0x20, 0x00, 0x80, 0x04, pin[0], pin[1], pin[2], pin[3] };
            return await CommandApdu(smartCardConnection, apdu);
        }

        private static bool TryGetKihon4InformationHeader(IReadOnlyList<byte> response, out byte contentLength)
        {
            var position = 2;
            var tmp = response.Skip(position).Take(3).ToArray();
            contentLength = (byte)(tmp[2] + 5);

            return true;
        }

        private static bool TryGetKihon4Information(IReadOnlyList<byte> response, out string name, out string address, out string dateOfBirth, out string sex)
        {
            // skip header and get content length
            var position = 2;
            var tmp = response.Skip(position).Take(3).ToArray();
            position += 3;
            var contentLength = tmp[2];

            // skip unknown
            tmp = response.Skip(position).Take(3).ToArray();
            position += 3;
            var unknownLength = tmp[2];
            var unknownBytes = response.Skip(position).Take(unknownLength).ToArray();
            position += unknownLength;

            // get name
            tmp = response.Skip(position).Take(3).ToArray();
            position += 3;
            var nameLength = tmp[2];
            var nameBytes = response.Skip(position).Take(nameLength).ToArray();
            position += nameLength;
            name = Encoding.UTF8.GetString(nameBytes);

            // get address
            tmp = response.Skip(position).Take(3).ToArray();
            position += 3;
            var addressLength = tmp[2];
            var addressBytes = response.Skip(position).Take(addressLength).ToArray();
            position += addressLength;
            address = Encoding.UTF8.GetString(addressBytes);

            // get date of birth
            tmp = response.Skip(position).Take(3).ToArray();
            position += 3;
            var dateOfBirthLength = tmp[2];
            var dateOfBirthBytes = response.Skip(position).Take(dateOfBirthLength).ToArray();
            position += dateOfBirthLength;
            dateOfBirth = Encoding.ASCII.GetString(dateOfBirthBytes);

            // get sex
            tmp = response.Skip(position).Take(3).ToArray();
            position += 3;
            var sexLength = tmp[2];
            var sexBytes = response.Skip(position).Take(sexLength).ToArray();
            position += dateOfBirthLength;
            sex = Encoding.ASCII.GetString(sexBytes);

            return true;
        }

        private static bool TryGetMyNumber(IReadOnlyList<byte> response, out string myNumber)
        {
            var position = 0;
            var tmp = response.Skip(position).Take(3).ToArray();
            position += 3;
            var myNumberLength = tmp[2];
            var myNumberBytes = response.Skip(position).Take(myNumberLength).ToArray();
            myNumber = Encoding.ASCII.GetString(myNumberBytes);

            return true;
        }

        private static bool TryGetBinaryLength(IReadOnlyList<byte> response, out int length)
        {
            length = 0;
            
            if (!IsSuccess(response.Skip(1).Take(2).ToArray()))
                return false;

            length = response[0];
            return true;
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
