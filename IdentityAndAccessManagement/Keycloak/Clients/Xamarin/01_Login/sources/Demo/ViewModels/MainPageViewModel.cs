using System;

using Prism.Commands;
using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    public sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Fields

        private readonly ILoginService _LoginService;

        #endregion

        #region Constructors

        public MainPageViewModel(INavigationService navigationService,
                                 ILoginService loginService,
                                 ILoggingService loggingService)
            : base(navigationService, loggingService)
        {
            this.Title = "Main";

            this._LoginService = loginService;

            Xamarin.Forms.Device.StartTimer(TimeSpan.FromSeconds(1), () =>
            {
                var expiration = this._LoginService.AccessTokenExpiration;
                if (expiration == null)
                {
                    this.AccessTokenExpire = "Expired";
                    return true;
                }

                var totalSeconds = (expiration - DateTime.UtcNow).Value.TotalSeconds;
                this.AccessTokenExpire = totalSeconds > 0 ? $"{(int)totalSeconds} sec" : "Expired";
                return true;
            });
        }

        #endregion

        #region Methods

        #region Event Handlers

        public override void OnNavigatedTo(INavigationParameters parameters)
        {
            this.LoggingService.Info($"Navigated to {this.GetType().Name}");

            base.OnNavigatedTo(parameters);
        }

        #endregion

        #region Helpers
        
        private async void RefreshToken()
        {
            await this._LoginService.RefreshToken();
        }

        #endregion

        #endregion

        #region IMainPageViewModel Members

        private string _AccessTokenExpire;

        public string AccessTokenExpire
        {
            get => this._AccessTokenExpire;
            private set => this.SetProperty(ref this._AccessTokenExpire, value);
        }

        private DelegateCommand _RefreshCommand;

        public DelegateCommand RefreshCommand => this._RefreshCommand ?? (this._RefreshCommand = new DelegateCommand(this.RefreshToken));

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
