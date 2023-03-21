using System.Threading.Tasks;

using Demo.Models;

namespace Demo.Services.Interfaces
{

    public interface IStorageService
    {

        Task ClearAuthenticationResult();

        Task<AuthenticationResult> GetAuthenticationResult();

        Task SetAuthenticationResult(AuthenticationResult authenticationResult);

    }

}
