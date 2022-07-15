using System.IO;
using Prism.Commands;
using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

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
            : base(navigationService)
        {
            this._LoggingService = loggingService;
            Title = "Main Page";
        }

        #endregion

        #region IMainPageViewModel Members

        private DelegateCommand<string> _LoggingCommand;

        public DelegateCommand<string> LoggingCommand
        {
            get
            {
                return this._LoggingCommand ?? (this._LoggingCommand = new DelegateCommand<string>(s =>
                    {
                        switch (s)
                        {
                            case "Trace":
                                this._LoggingService.Trace($"This is {s}");
                                break;
                            case "Debug":
                                this._LoggingService.Debug($"This is {s}");
                                break;
                            case "Info":
                                this._LoggingService.Info($"This is {s}");
                                break;
                            case "Warn":
                                this._LoggingService.Warn($"This is {s}");
                                break;
                            case "Error":
                                this._LoggingService.Error($"This is {s}");
                                break;
                            case "Fatal":
                                this._LoggingService.Fatal($"This is {s}");
                                break;
                        }
                    }, s => true));
            }
        }
        
        private DelegateCommand _LoadCommand;

        public DelegateCommand LoadCommand
        {
            get
            {
                return this._LoadCommand ?? (this._LoadCommand = new DelegateCommand(() =>
                {
                    var path = this._LoggingService.GetCurrentLogFilePath();
                    if (!string.IsNullOrWhiteSpace(path) && File.Exists(path))
                        this.Log = File.ReadAllText(path);
                }, () => true));
            }
        }

        private string _Log;

        public string Log
        {
            get => this._Log;
            set => SetProperty(ref this._Log, value);
        }

        private string _Title;

        public string Title
        {
            get => this._Title;
            set => SetProperty(ref this._Title, value);
        }

        #endregion

    }

}
