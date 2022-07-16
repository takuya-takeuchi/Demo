using System.IO;
using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    public sealed class ShowLogsViewModel : ViewModelBase, IShowLogsViewModel
    {

        #region Fields

        private readonly ILoggingService _LoggingService;

        #endregion

        #region Constructors

        public ShowLogsViewModel(INavigationService navigationService,
                                 ILoggingService loggingService)
            : base(navigationService)
        {
            this.Title = "Logs";

            this._LoggingService = loggingService;
        }

        #endregion

        #region Properties

        private string _Logs;

        public string Logs
        {
            get => this._Logs;
            private set => SetProperty(ref this._Logs, value);
        }

        #endregion

        #region Methods

        #region Event Handlers

        public override void OnNavigatedTo(INavigationParameters parameters)
        {
            var path = this._LoggingService.GetCurrentLogFilePath();
            if (!string.IsNullOrEmpty(path) && File.Exists(path))
                this.Logs = File.ReadAllText(path);

            base.OnNavigatedTo(parameters);
        }

        #endregion

        #endregion

    }

}
