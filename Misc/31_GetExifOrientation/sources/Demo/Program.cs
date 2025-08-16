using System;
using System.Drawing;

using ExifLib;
using NLog;

namespace Demo
{

    internal sealed class Program
    {

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Methods

        private static void Main(string[] args)
        {
            if (args.Length != 1)
            {
                Logger.Error("Usage: Demo.exe </path/to/image/file>");
                return;
            }

            var inputPath = args[0];
            using (var reader = new ExifReader(inputPath))
            {
                if (reader.GetTagValue(ExifTags.Orientation, out ushort value))
                    Logger.Info($"ExifLib found Orientation Tag: {value}");
                else
                    Logger.Info("ExifLib does not find Orientation Tag");
            }

            using (var original = new Bitmap(inputPath))
            {
                try
                {
                    var item = original.GetPropertyItem((int)ExifTags.Orientation);
                    var value = BitConverter.ToInt16(item.Value, 0);
                    Logger.Info($"System.Drawing found Orientation Tag: {value}");
                }
                catch (Exception e)
                {
                    Logger.Info("System.Drawing does not find Orientation Tag");
                }
            }
        }

        #endregion

    }

}