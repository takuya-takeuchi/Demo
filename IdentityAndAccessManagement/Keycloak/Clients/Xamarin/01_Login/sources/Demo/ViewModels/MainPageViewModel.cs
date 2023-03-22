using System;
using System.Linq;
using Demo.Models;
using Prism.Commands;
using Prism.Events;
using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    public sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Fields

        private readonly IEventAggregator _EventAggregator;

        private readonly ILoginService _LoginService;

        private readonly ITokenTimerService _TokenTimerService;

        #endregion

        #region Constructors

        public MainPageViewModel(INavigationService navigationService,
                                 IEventAggregator eventAggregator,
                                 ILoginService loginService,
                                 ILoggingService loggingService,
                                 ITokenTimerService tokenTimerService)
            : base(navigationService, loggingService)
        {
            this.Title = "Main";

            this._EventAggregator = eventAggregator;
            this._LoginService = loginService;
            this._TokenTimerService = tokenTimerService;

            this._EventAggregator.GetEvent<TokenTimerElapsedEvent>().Subscribe(this.OnTokenTimerElapsedEvent, ThreadOption.UIThread);
        }

        #endregion

        #region Methods

        #region Overrides

        public override async void OnNavigatedTo(INavigationParameters parameters)
        {
            this.LoggingService.Info($"Navigated to {this.GetType().Name}");

            // from application start up
            if (!parameters.Any())
                await this.NavigationService.NavigateAsync("LoginPage");
            else
            {
                var authenticationResult = parameters.GetValue<AuthenticationResult>(nameof(AuthenticationResult));
                this._TokenTimerService.SetAuthenticationResult(authenticationResult);
            }

            base.OnNavigatedTo(parameters);
        }

        #endregion

        #region Event Handlers
        
        private void OnTokenTimerElapsedEvent(DateTimeOffset? expiration)
        {
            if (expiration == null)
            {
                this.AccessTokenExpire = "Expired";
                return;
            }

            var totalSeconds = (expiration - DateTime.UtcNow).Value.TotalSeconds;
            this.AccessTokenExpire = totalSeconds > 0 ? $"{(int)totalSeconds} sec" : "Expired";
        }

        #endregion

        #region Helpers

        private async void RefreshToken()
        {
            var refreshToken = await this._LoginService.RefreshToken();
            if (refreshToken == null)
            {
                await this.NavigationService.NavigateAsync("LoginPage");
            }
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
