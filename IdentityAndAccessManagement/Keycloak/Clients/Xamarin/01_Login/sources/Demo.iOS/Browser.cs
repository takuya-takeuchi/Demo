using System;
using System.Threading;
using System.Threading.Tasks;

using AuthenticationServices;
using Foundation;
using UIKit;

using Xamarin.Forms;

using IdentityModel.OidcClient.Browser;


[assembly: Dependency(typeof(Demo.iOS.Browser))]
namespace Demo.iOS
{

    internal sealed class Browser : IBrowser
    {

        #region Fields

        private ASWebAuthenticationSession _ASWebAuthenticationSession;

        #endregion

        #region Methods

        #region Helpers

        private static BrowserResult CreateBrowserResult(NSUrl callbackUrl, NSError error)
        {
            if (error == null)
                return new BrowserResult
                {
                    ResultType = BrowserResultType.Success,
                    Response = callbackUrl.AbsoluteString
                };

            if (error.Code == (long)ASWebAuthenticationSessionErrorCode.CanceledLogin)
                return new BrowserResult
                {
                    ResultType = BrowserResultType.UserCancel,
                    Error = error.ToString()
                };

            return new BrowserResult
            {
                ResultType = BrowserResultType.UnknownError,
                Error = error.ToString()
            };
        }

        #endregion

        #endregion
        
        #region IBrowser Members

        public Task<BrowserResult> InvokeAsync(BrowserOptions options, CancellationToken cancellationToken = new CancellationToken())
        {
            var tcs = new TaskCompletionSource<BrowserResult>();

            try
            {
                this._ASWebAuthenticationSession = new ASWebAuthenticationSession(
                    new NSUrl(options.StartUrl),
                    new NSUrl(options.EndUrl).Scheme,
                    (callbackUrl, error) =>
                    {
                        tcs.SetResult(CreateBrowserResult(callbackUrl, error));
                        this._ASWebAuthenticationSession.Dispose();
                    });

                // iOS 13 requires the PresentationContextProvider set
                if (UIDevice.CurrentDevice.CheckSystemVersion(13, 0))
                    this._ASWebAuthenticationSession.PresentationContextProvider = new PresentationContextProviderToSharedKeyWindow();

                this._ASWebAuthenticationSession.Start();
            }
            catch (Exception ex)
            {
                throw;
            }

            return tcs.Task;
        }

        #endregion
        
        private sealed class PresentationContextProviderToSharedKeyWindow : NSObject, IASWebAuthenticationPresentationContextProviding
        {

            #region IASWebAuthenticationPresentationContextProviding Members
            
            public UIWindow GetPresentationAnchor(ASWebAuthenticationSession session)
            {
                return UIApplication.SharedApplication.KeyWindow;
            }

            #endregion

        }

    }

}