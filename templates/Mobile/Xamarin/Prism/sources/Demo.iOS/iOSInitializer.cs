using Prism;
using Prism.Ioc;

namespace Demo.iOS
{

    public sealed class iOSInitializer : IPlatformInitializer
    {

        #region Methods

        public void RegisterTypes(IContainerRegistry containerRegistry)
        {
            // Register any platform specific implementations
        }

        #endregion

    }

}