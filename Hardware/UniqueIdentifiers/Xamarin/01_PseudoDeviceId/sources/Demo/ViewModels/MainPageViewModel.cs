using Prism.Commands;
using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    public sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Fields

        private readonly IDeviceService _DeviceService;

        #endregion

        #region Constructors

        public MainPageViewModel(INavigationService navigationService,
                                 ILoggingService loggingService,
                                 IDeviceService deviceService)
            : base(navigationService, loggingService)
        {
            this.Title = "Main Page";
            this._DeviceService = deviceService;

            this.Key = this._DeviceService.GetKey();
            this.LoggingService.Info(this.Key);
        }

        #endregion

        #region Properties

        private string _Key;

        public string Key
        {
            get => this._Key;
            private set => this.SetProperty(ref this._Key, value);
        }

        private DelegateCommand _ReloadCommand;

        public DelegateCommand ReloadCommand
        {
            get
            {
                return this._ReloadCommand ?? (this._ReloadCommand = new DelegateCommand(() =>
                {
                    this.Key = this._DeviceService.GetKey();
                }));
            }
        }

        private DelegateCommand _ResetCommand;

        public DelegateCommand ResetCommand
        {
            get
            {
                return this._ResetCommand ?? (this._ResetCommand = new DelegateCommand(() =>
                {
                    this._DeviceService.Reset();
                    this.Key = this._DeviceService.GetKey();
                }));
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