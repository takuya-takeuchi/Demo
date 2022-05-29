using System;
using Microsoft.Practices.Unity;
using Prism.Unity;
using Xamarin.Forms.Portable8.ViewModels;
using Xamarin.Forms.Portable8.Views;

namespace Xamarin.Forms.Portable8
{
    public sealed class Bootstrapper : UnityBootstrapper
    {
        [Obsolete]
        protected override Xamarin.Forms.Page CreateMainPage()
        {
            return this.Container.Resolve<Views.MainPage>();
        }

        protected override void RegisterTypes()
        {
            // ViewModels
            this.Container.RegisterType<MainPageViewModel>();
            this.Container.RegisterType<NavigationPageViewModel>();
            this.Container.RegisterType<FirstPageViewModel>();
            this.Container.RegisterType<NextPageViewModel>();

            // Views
            // Maybe, the following codes are meaningless because NavigationSerivce is not used.
            //this.Container.RegisterType<MainPage>();
            //this.Container.RegisterType<Views.NavigationPage>();
            //this.Container.RegisterType<Views.FirstPage>();
            //this.Container.RegisterType<Views.NextPage>();

            //this.Container.RegisterTypeForNavigation<Views.NavigationPage>();
            //this.Container.RegisterTypeForNavigation<FirstPage>();
            //this.Container.RegisterTypeForNavigation<NextPage>();
        }

        protected override void OnInitialized()
        {
            // I guess it is meaningless.
            //this.NavigationService.Navigate("/NavigationPage/MainPage");
        }
    }
}