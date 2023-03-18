using Prism.Commands;
using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    public sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Constructors

        public MainPageViewModel(INavigationService navigationService,
                                 ILoggingService loggingService)
            : base(navigationService, loggingService)
        {
        }

        #endregion

    }

}
