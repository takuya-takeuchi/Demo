using System.IO;
using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;
using Prism.Commands;

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

        #region Properties
        #endregion

        #region Methods

        #region Event Handlers

        public override void OnNavigatedTo(INavigationParameters parameters)
        {
            base.OnNavigatedTo(parameters);
        }

        #endregion

        #endregion

    }

}
