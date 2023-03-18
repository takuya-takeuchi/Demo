using Prism;
using Prism.Ioc;

using Demo.iOS.Services;
using Demo.Services.Interfaces;

namespace Demo.iOS
{

    public sealed class iOSInitializer : IPlatformInitializer
    {

        #region Methods

        public void RegisterTypes(IContainerRegistry containerRegistry)
        {
            containerRegistry.Register<IMediaService, MediaService>();
        }

        #endregion

    }

}