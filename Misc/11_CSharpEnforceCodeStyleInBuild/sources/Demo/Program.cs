using NLog;

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
            string message = "Start";
            Logger.Info(message);
        }

        #endregion

    }

}
