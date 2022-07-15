using Prism.Navigation;

namespace Demo.ViewModels
{

    public sealed class MainPageViewModel : ViewModelBase
    {

        #region Constructors

        public MainPageViewModel(INavigationService navigationService)
            : base(navigationService)
        {
            Title = "Main Page";
        }

        #endregion

    }

}
