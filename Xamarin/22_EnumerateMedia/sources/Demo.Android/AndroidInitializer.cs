using Prism;
using Prism.Ioc;

using Demo.Droid.Services;
using Demo.Services.Interfaces;

namespace Demo.Droid
{

    public sealed class AndroidInitializer : IPlatformInitializer
    {

        #region IPlatformInitializer Members

        public void RegisterTypes(IContainerRegistry containerRegistry)
        {
            containerRegistry.Register<IMediaService, MediaService>();
        }

        #endregion

    }

}