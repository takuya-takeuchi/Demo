using System.IO;
using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;
using Prism.Commands;

namespace Demo.ViewModels
{

    public sealed class LogPageViewModel : TabbedPageViewModelBase, ILogPageViewModel
    {

        #region Fields

        private readonly ILoggingService _LoggingService;

        #endregion

        #region Constructors

        public LogPageViewModel(INavigationService navigationService,
                                ILoggingService loggingService)
            : base(navigationService, loggingService)
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

        #region Overrides

        protected override void OnActivated()
        {
            this.LoggingService.Info($"{nameof(LogPageViewModel)} is activated");

            var path = this._LoggingService.GetCurrentLogFilePath();
            if (!string.IsNullOrEmpty(path) && File.Exists(path))
                this.Logs = File.ReadAllText(path);

            base.OnActivated();
        }

        protected override void OnDeactivated()
        {
            this.LoggingService.Info($"{nameof(LogPageViewModel)} is deactivated");
            base.OnDeactivated();
        }

        #endregion

        #endregion

    }

}
