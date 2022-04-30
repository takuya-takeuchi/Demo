using Xamarin.Camera.ViewModels.Interfaces;

namespace Xamarin.Camera.DesignTimes
{

    public sealed class SettingPageViewModel : ISettingPageViewModel
    {

        #region Constructors

        public SettingPageViewModel()
        {
            this.Title = "Setting";
        }

        #endregion

        #region Properties

        public string Title
        {
            get;
        }

        #endregion

    }

}
