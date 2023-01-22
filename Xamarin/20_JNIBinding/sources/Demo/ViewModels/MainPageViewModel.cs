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
                                 INativeService nativeService,
                                 ILoggingService loggingService)
            : base(navigationService)
        {
            this._LoggingService = loggingService;
            var message = nativeService.Message();
            this.Title = $"Main Page ({message})";
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

        private DelegateCommand _ShowLogCommand;

        public DelegateCommand ShowLogCommand
        {
            get
            {
                return this._ShowLogCommand ?? (this._ShowLogCommand = new DelegateCommand(() =>
                {
                    this.NavigationService.NavigateAsync("ShowLogs");
                }));
            }
        }

        #endregion

    }

}
