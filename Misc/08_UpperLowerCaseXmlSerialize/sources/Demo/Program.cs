using System.Xml.Serialization;
using NLog;

namespace DemoCSharp
{

    internal sealed class Program
    {

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Methods

        private static void Main()
        {
            TestDeserialize<Test>("TestUpper.xml");
            TestDeserialize<Test>("TestLower.xml");
            TestDeserialize<TEST>("TestUpper.xml");
            TestDeserialize<TEST>("TestLower.xml");
        }

        #region Helpers

        private static void TestDeserialize<T>(string filename)
        {
            try
            {
                var serializer = new XmlSerializer(typeof(T));
                using var fileStream = new FileStream(filename, FileMode.Open, FileAccess.Read);
                serializer.Deserialize(fileStream);
                Logger.Info($"Succeed to deserialize {typeof(T).FullName} from {filename}");
            }
            catch
            {
                Logger.Error($"Failed to deserialize {typeof(T).FullName} from {filename}");
            }
        }

        #endregion

        #endregion

    }

    public sealed class Test
    {

        public int Member
        {
            get;
            set;
        }

    }

    public sealed class TEST
    {

        public int Member
        {
            get;
            set;
        }

    }

}
