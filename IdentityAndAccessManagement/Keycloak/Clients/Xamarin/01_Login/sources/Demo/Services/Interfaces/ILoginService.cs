using System.Threading.Tasks;
using IdentityModel.OidcClient;

namespace Demo.Services.Interfaces
{

    public interface ILoginService
    {

        Task<LoginResult> Login();

        Task Logout();

    }

}
