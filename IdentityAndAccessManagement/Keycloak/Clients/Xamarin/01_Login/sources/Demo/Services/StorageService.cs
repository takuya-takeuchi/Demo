using System.Threading.Tasks;

using Xamarin.Essentials;

using Demo.Services.Interfaces;
using Demo.Models;

namespace Demo.Services
{

    public sealed class StorageService : IStorageService
    {

        #region Fields

        private readonly ILoggingService _LoggingService;

        #endregion

        #region Constructors

        public StorageService(ILoggingService loggingService)
        {
            this._LoggingService = loggingService;
        }

        #endregion

        #region IStorageService Members

        public Task ClearAuthenticationResult()
        {
            SecureStorage.Remove(nameof(AuthenticationResult));
            return Task.CompletedTask;
        }

        public async Task<AuthenticationResult> GetAuthenticationResult()
        {
            var json = await SecureStorage.GetAsync(nameof(AuthenticationResult));
            return json == null ? null : System.Text.Json.JsonSerializer.Deserialize<AuthenticationResult>(json);
        }

        public async Task SetAuthenticationResult(AuthenticationResult authenticationResult)
        {
            var json = System.Text.Json.JsonSerializer.Serialize(authenticationResult);
            await SecureStorage.SetAsync(nameof(AuthenticationResult), json);
        }

        #endregion

    }

}