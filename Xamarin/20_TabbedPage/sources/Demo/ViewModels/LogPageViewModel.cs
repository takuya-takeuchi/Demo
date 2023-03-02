using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    public sealed class LogPageViewModel : ViewModelBase, ILogPageViewModel
    {

        #region Fields

        private readonly ILoggingService _LoggingService;

        #endregion

        #region Constructors

        public LogPageViewModel(INavigationService navigationService,
                                ILoggingService loggingService)
            : base(navigationService)
        {
            this.Title = "Logs";

            this._LoggingService = loggingService;
        }

        #endregion

    }

}
