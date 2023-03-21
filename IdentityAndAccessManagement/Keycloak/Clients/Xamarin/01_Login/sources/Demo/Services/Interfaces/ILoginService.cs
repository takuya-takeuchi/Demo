using System;
using System.Threading.Tasks;

using Demo.Models;

namespace Demo.Services.Interfaces
{

    public interface ILoginService
    {

        DateTimeOffset? AccessTokenExpiration
        {
            get;
        }

        Task<AuthenticationResult> Login();

        Task Logout();

        Task<AuthenticationResult> RefreshToken();

    }

}
