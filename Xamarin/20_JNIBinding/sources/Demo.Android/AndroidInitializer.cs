using Demo.Droid.Services;
using Demo.Services.Interfaces;
using Prism;
using Prism.Ioc;
using Xamarin.Essentials.Implementation;
using Xamarin.Essentials.Interfaces;

namespace Demo.Droid
{

    public sealed class AndroidInitializer : IPlatformInitializer
    {

        #region Methods

        public void RegisterTypes(IContainerRegistry containerRegistry)
        {
            containerRegistry.RegisterSingleton<INativeService, NativeService>();
        }

        #endregion

    }

}