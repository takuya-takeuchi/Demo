using System;
using System.IO;

using Microsoft.Office.Interop.Word;

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
            var template = args[0];
            var imagePath = args[1];
            var outputDocPath = args[2];

            var missing = Type.Missing;
            Application app = null;;
            Document doc = null;

            try
            {
                app = new Application();
                doc = app.Documents.Open(template, ref missing, true);

                foreach (Bookmark b in doc.Bookmarks)
                {
                    // Embed image without link
                    if (b.Name == "ImageField1")
                    {
                        var shape = (InlineShape)doc.Bookmarks["ImageField1"].Range.InlineShapes.AddPicture(imagePath, false, true);
                        shape.ScaleWidth = 50f;
                        shape.ScaleHeight = 50f;
                    }
                }

                // Can not overwrite by SaveAs
                if (File.Exists(outputDocPath))
                    File.Delete(outputDocPath);

                doc.SaveAs(outputDocPath);
            }
            catch (Exception e)
            {
                Logger.Error(e, null, "Failed to process");
            }
            finally
            {
                var docClose = (_Document)doc;
                docClose?.Close();
            }
        }

        #endregion

    }

}