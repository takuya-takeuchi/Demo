using System;

using Xamarin.Forms;

using Prism.Events;

using Demo.Services.Interfaces;
using Demo.Models;

namespace Demo.Services
{

    public sealed class TokenTimerService : ITokenTimerService
    {

        #region Fields

        private readonly IEventAggregator _EventAggregator;

        private readonly ILoggingService _LoggingService;

        private AuthenticationResult _AuthenticationResult;

        #endregion

        #region Constructors

        public TokenTimerService(IEventAggregator eventAggregator,
                                 ILoggingService loggingService)
        {
            this._EventAggregator = eventAggregator;
            this._LoggingService = loggingService;

            Device.StartTimer(TimeSpan.FromSeconds(1), () =>
            {
                this._EventAggregator.GetEvent<TokenTimerElapsedEvent>().Publish(this._AuthenticationResult?.AccessTokenExpiration);
                return true;
            });
        }

        #endregion

        #region ITokenTimerService Members

        public void SetAuthenticationResult(AuthenticationResult authenticationResult)
        {
            this._AuthenticationResult = authenticationResult;
        }

        #endregion

    }

}