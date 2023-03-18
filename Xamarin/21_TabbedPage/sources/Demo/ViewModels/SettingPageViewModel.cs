using Prism.Navigation;

using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    public sealed class SettingPageViewModel : ViewModelBase, ISettingPageViewModel
    {

        #region Constructors

        public SettingPageViewModel(INavigationService navigationService)
            : base(navigationService)
        {
            this.Title = "Setting";
        }

        #endregion

    }

}
