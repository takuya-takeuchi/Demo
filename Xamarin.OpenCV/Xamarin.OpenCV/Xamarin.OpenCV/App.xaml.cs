using Prism;
using Prism.Ioc;
using Xamarin.Essentials.Implementation;
using Xamarin.Essentials.Interfaces;
using Xamarin.Forms;
using Xamarin.Forms.Xaml;
using Xamarin.OpenCV.Services;
using Xamarin.OpenCV.ViewModels;
using Xamarin.OpenCV.Views;

[assembly: XamlCompilation(XamlCompilationOptions.Compile)]
namespace Xamarin.OpenCV
{

    public partial class App
    {

        #region Constructors

        /* 
         * The Xamarin Forms XAML Previewer in Visual Studio uses System.Activator.CreateInstance.
         * This imposes a limitation in which the App class must have a default constructor. 
         * App(IPlatformInitializer initializer = null) cannot be handled by the Activator.
         */
        public App() : this(null) { }

        public App(IPlatformInitializer initializer) : base(initializer) { }

        #endregion

        #region Properties
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

            // Services
            containerRegistry.RegisterSingleton<IOpenCVService, OpenCvService>();
            //containerRegistry.RegisterSingleton<IOpenCVService, Services.Mocks.OpenCvService>();

            containerRegistry.RegisterForNavigation<NavigationPage>();
            containerRegistry.RegisterForNavigation<MainPage, MainPageViewModel>();
        }

        #endregion

        #endregion

    }

}

