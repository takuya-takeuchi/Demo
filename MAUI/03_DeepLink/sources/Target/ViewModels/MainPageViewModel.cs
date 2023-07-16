using Target.ViewModels.Interfaces;

namespace Target.ViewModels
{

    internal sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Constructors

        public MainPageViewModel(INavigationService navigationService)
            : base(navigationService)
        {
        }

        #endregion

    }

}