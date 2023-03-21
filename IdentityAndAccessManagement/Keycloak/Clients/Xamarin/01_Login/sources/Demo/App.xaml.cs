using Xamarin.Essentials.Implementation;
using Xamarin.Essentials.Interfaces;

using Prism;
using Prism.Ioc;

using Demo.Services;
using Demo.Services.Interfaces;
using Demo.ViewModels;
using Demo.Views;

namespace Demo
{

    public partial class App
    {

        #region Constructors

        public App(IPlatformInitializer initializer)
            : base(initializer)
        {
        }

        #endregion

        #region Methods

        #region Overrids

        protected override async void OnInitialized()
        {
            InitializeComponent();
            
            await this.NavigationService.NavigateAsync("LoginPage");
        }

        protected override void RegisterTypes(IContainerRegistry containerRegistry)
        {
            containerRegistry.RegisterSingleton<IAppInfo, AppInfoImplementation>();

            containerRegistry.RegisterSingleton<ILoggingService, LoggingService>();
            containerRegistry.RegisterSingleton<ILoginService, LoginService>();

            //containerRegistry.RegisterForNavigation<NavigationPage>();
            containerRegistry.RegisterForNavigation<MainPage, MainPageViewModel>();
            containerRegistry.RegisterForNavigation<LoginPage, LoginPageViewModel>();
            containerRegistry.RegisterForNavigation<ShowLogs, ShowLogsViewModel>();
        }

        #endregion

        #endregion

    }

}
