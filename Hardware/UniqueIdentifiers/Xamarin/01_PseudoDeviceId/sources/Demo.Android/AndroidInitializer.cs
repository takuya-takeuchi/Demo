using Prism;
using Prism.Ioc;

using Demo.Droid.Services;
using Demo.Services.Interfaces;

namespace Demo.Droid
{

    public sealed class AndroidInitializer : IPlatformInitializer
    {

        #region Methods

        public void RegisterTypes(IContainerRegistry containerRegistry)
        {
            // Register any platform specific implementations
            containerRegistry.RegisterSingleton<IDeviceService, DeviceService>();
        }

        #endregion

    }

}