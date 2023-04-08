using Prism;
using Prism.Ioc;

namespace Demo.Droid
{

    public sealed class AndroidInitializer : IPlatformInitializer
    {

        #region Methods

        public void RegisterTypes(IContainerRegistry containerRegistry)
        {
            // Register any platform specific implementations
        }

        #endregion

    }

}