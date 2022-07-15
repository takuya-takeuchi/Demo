using Prism;
using Prism.Ioc;
using Xamarin.Essentials.Implementation;
using Xamarin.Essentials.Interfaces;
using Xamarin.Forms;

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

            await NavigationService.NavigateAsync("NavigationPage/MainPage");
        }

        protected override void RegisterTypes(IContainerRegistry containerRegistry)
        {
            containerRegistry.RegisterSingleton<IAppInfo, AppInfoImplementation>();

            containerRegistry.RegisterInstance<IZeroMQSubscribeService>(new ZeroMQSubscribeService());

            containerRegistry.RegisterForNavigation<NavigationPage>();
            containerRegistry.RegisterForNavigation<MainPage, MainPageViewModel>();
        }

        #endregion

        #endregion

    }

}
