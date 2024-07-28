using MaterialTemplate.ViewModels.Interfaces;

namespace MaterialTemplate.DesignTimes
{

    public sealed class MainWindowViewModel : IMainWindowViewModel
    {

        #region IMainWindowViewModel Members

        public string Title
        {
            get
            {
                return "Material Design Project Template (DesignTimes)";
            }
        }

        #endregion

    }

}
