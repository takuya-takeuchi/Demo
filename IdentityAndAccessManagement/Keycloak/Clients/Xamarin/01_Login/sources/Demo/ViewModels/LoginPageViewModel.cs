using System;
using System.Threading.Tasks;

using Prism.Commands;
using Prism.Navigation;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    public sealed class LoginPageViewModel : ViewModelBase, ILoginPageViewModel
    {

        #region Fields

        private readonly ILoginService _LoginService;

        #endregion

        #region Constructors

        public LoginPageViewModel(INavigationService navigationService,
                                  ILoggingService loggingService,
                                  ILoginService loginService)
            : base(navigationService, loggingService)
        {
            this.Title = "Login";

            this._LoginService = loginService;
        }

        #endregion

        #region Methods

        #region Helpers

        private async void Login()
        {
            try
            {
                await this._LoginService.Login();
                await this.NavigationService.NavigateAsync("/MainPage");
            }
            catch (Exception ex)
            {
                this.LoggingService.Error(ex, null, "Failed to login");
            }
        }

        private async void Logout()
        {
            try
            {
                await this._LoginService.Logout();
            }
            catch (Exception ex)
            {
                this.LoggingService.Error(ex, null, "Failed to logout");
            }

            this.IsLoggedIn = false;
        }

        #endregion

        #endregion

        #region ILoginPageViewModel Members

        private bool _IsLoggedIn;

        public bool IsLoggedIn
        {
            get => this._IsLoggedIn;
            private set => this.SetProperty(ref this._IsLoggedIn, value);
        }

        private DelegateCommand _LoginCommand;

        public DelegateCommand LoginCommand => this._LoginCommand ?? (this._LoginCommand = new DelegateCommand(this.Login));

        private DelegateCommand _LogoutCommand;

        public DelegateCommand LogoutCommand => this._LogoutCommand ?? (this._LogoutCommand = new DelegateCommand(this.Logout));

        #endregion

    }

}
