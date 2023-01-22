using System.IO;
using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;
using Prism.Commands;

namespace Demo.ViewModels
{

    public sealed class ShowLogsViewModel : ViewModelBase, IShowLogsViewModel
    {

        #region Fields

        private readonly ILoggingService _LoggingService;

        #endregion

        #region Constructors

        public ShowLogsViewModel(INavigationService navigationService,
                                 INativeService nativeService,
                                 ILoggingService loggingService)
            : base(navigationService)
        {
            var message = nativeService.Message();
            this.Title = $"Logs ({message})";

            this._LoggingService = loggingService;
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
                    var path = this._LoggingService.GetCurrentLogFilePath();
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
            var path = this._LoggingService.GetCurrentLogFilePath();
            if (!string.IsNullOrEmpty(path) && File.Exists(path))
                this.Logs = File.ReadAllText(path);

            base.OnNavigatedTo(parameters);
        }

        #endregion

        #endregion

    }

}
