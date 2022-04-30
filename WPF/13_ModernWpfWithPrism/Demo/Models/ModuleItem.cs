
namespace Demo.Models
{

    internal sealed class ModuleItem
    {

        #region Constructors

        public ModuleItem(string moduleName, string text)
        {
            this.ModuleName = moduleName;
            this.Text = text;
        }

        #endregion

        #region Properties

        public string ModuleName
        {
            get;
        }

        public string Text
        {
            get;
        }

        #endregion

    }

}
