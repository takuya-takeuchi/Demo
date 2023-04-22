using System;

using Microsoft.Office.Interop.Word;

using NLog;

namespace Demo
{

    internal sealed class Program
    {

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        private static readonly string FieldPrefix = " MERGEFIELD";

        #endregion

        #region Methods

        private static void Main(string[] args)
        {
            var template = args[0];
            var text = args[1];
            var imagePath = args[2];
            var outputDocPath = args[3];

            var missing = Type.Missing;
            Application app = null;;
            Document doc = null;

            try
            {
                app = new Application();
                doc = app.Documents.Open(template, ref missing, true);

                foreach (Field field in doc.Fields)
                {
                    var range = field.Code;
                    var fieldText = range.Text;
                    if (fieldText.StartsWith(FieldPrefix))
                    {
                        var endMerge = fieldText.IndexOf("\\");
                        var fieldName = fieldText.Substring(FieldPrefix.Length, endMerge - FieldPrefix.Length);

                        fieldName = fieldName.Trim();
                        if (fieldName == "TextField1")
                        {
                            field.Select();
                            app.Selection.TypeText(text);
                        }
                    }
                }

                foreach (Bookmark b in doc.Bookmarks)
                {
                    // Embed image without link
                    if (b.Name == "ImageField1")
                        doc.Bookmarks["ImageField1"].Range.InlineShapes.AddPicture(imagePath, false, true);
                }

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