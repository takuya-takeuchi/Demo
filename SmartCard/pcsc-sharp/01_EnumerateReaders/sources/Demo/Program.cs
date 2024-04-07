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
            var contextFactory = ContextFactory.Instance;
            using var context = contextFactory.Establish(SCardScope.System);
            var readerNames = context.GetReaders();
            PrintAllReaders(readerNames);

            var groupNames = context.GetReaderGroups();
            PrintAllReaderGroups(groupNames);
            PrintReadersPerGroup(groupNames, context);

            Logger.Info("Press any key to exit.");
            Console.ReadKey();
        }

        #region Helpers

        private static void PrintReadersPerGroup(IEnumerable<string> groupNames, ISCardContext context)
        {
            foreach (var groupName in groupNames)
            {
                Logger.Info("Group " + groupName + " contains ");
                foreach (var readerName in context.GetReaders(new[] { groupName }))
                    Logger.Info("\t" + readerName);
            }
        }

        private static void PrintAllReaderGroups(IEnumerable<string> groupNames)
        {
            Logger.Info("Currently configured readers groups: ");
            foreach (var groupName in groupNames)
                Logger.Info("\t" + groupName);
        }

        private static void PrintAllReaders(IEnumerable<string> readerNames)
        {
            Logger.Info("Currently connected readers: ");
            foreach (var name in readerNames)
                Logger.Info("\t" + name);
        }

        #endregion

        #endregion

    }

}
