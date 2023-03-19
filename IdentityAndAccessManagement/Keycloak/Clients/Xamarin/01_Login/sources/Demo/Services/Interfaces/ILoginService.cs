using System.Threading.Tasks;

namespace Demo.Services.Interfaces
{

    public interface ILoginService
    {

        Task Login();

        Task Logout();

    }

}
