using System.Windows.Input;

using Prism.Commands;
using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;
using Xamarin.Forms;

namespace Demo.ViewModels
{

    public sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Fields

        private readonly ILoggingService _LoggingService;

        #endregion

        #region Constructors

        public MainPageViewModel(INavigationService navigationService,
                                 ILoggingService loggingService)
            : base(navigationService, loggingService)
        {
        }

        #endregion

    }

}
