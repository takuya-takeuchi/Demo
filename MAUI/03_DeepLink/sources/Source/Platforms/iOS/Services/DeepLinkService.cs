using System;
using System.Threading.Tasks;

using Microsoft.Maui.ApplicationModel;

using Source.Services.Interfaces;

namespace Source.Platforms.iOS.Services
{

    internal sealed class DeepLinkService : IDeepLinkService
    {

        #region IDeepLinkService Members

        public async Task Open()
        {
            var uri = new Uri("https://taktak.jp/buy/test.html");
            await Launcher.Default.OpenAsync(uri);
        }

        #endregion

    }

}
