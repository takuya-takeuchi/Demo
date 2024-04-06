using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using NLog;
using PCSC;

namespace Demo
{

    internal sealed class Program
    {

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Methods

        private static void Main()
        {
            using (var ctx = ContextFactory.Instance.Establish(SCardScope.User)) {
                var firstReader = ctx .GetReaders() .FirstOrDefault();
                if (firstReader == null) {
                    Logger.Error("No reader connected.");
                    return;
                }

                try
                {
                    using (var reader = ctx.ConnectReader(firstReader, SCardShareMode.Shared, SCardProtocol.Any)) {
                        var status = reader.GetStatus();

                        Logger.Info($"Reader names: {string.Join(", ", status.GetReaderNames())}");
                        Logger.Info($"Protocol: {status.Protocol}");
                        Logger.Info($"State: {status.State}");
                        Logger.Info($"ATR: {BitConverter.ToString(status.GetAtr() ?? new byte[0])}");
                    }

                    Logger.Info("Press any key to exit.");
                    Console.ReadKey();
                }
                catch (PCSC.Exceptions.RemovedCardException rce)
                {
                    Logger.Error(rce.Message);
                }
            }
        }

        #endregion

    }

}
