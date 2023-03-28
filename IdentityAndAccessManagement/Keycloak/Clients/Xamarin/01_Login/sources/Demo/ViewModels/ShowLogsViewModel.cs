using System.IO;

using Prism.Commands;
using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    public sealed class ShowLogsViewModel : ViewModelBase, IShowLogsViewModel
    {

        #region Constructors

        public ShowLogsViewModel(INavigationService navigationService,
                                 ILoggingService loggingService)
            : base(navigationService, loggingService)
        {
            this.Title = "Logs";
        }

        #endregion

        #region Properties

        private DelegateCommand _ClearCommand;

        public DelegateCommand ClearCommand
        {
            get
            {
                return this._ClearCommand ?? (this._ClearCommand = new DelegateCommand(() =>
                {
                    var path = this.LoggingService.GetCurrentLogFilePath();
                    if (!string.IsNullOrEmpty(path) && File.Exists(path))
                        File.Delete(path);
                    this.Logs = "";
                }));
            }
        }

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
            var path = this.LoggingService.GetCurrentLogFilePath();
            if (!string.IsNullOrEmpty(path) && File.Exists(path))
                this.Logs = File.ReadAllText(path);

            base.OnNavigatedTo(parameters);
        }

        #endregion

        #endregion

    }

}
