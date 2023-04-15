using System.Threading.Tasks;

namespace Demo.Services.Interfaces
{

    public interface IPermissionService
    {

        bool CheckCameraPermission();

        Task RequestCameraPermission();

    }

}
