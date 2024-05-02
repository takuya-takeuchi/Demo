using System.Security.Cryptography.X509Certificates;
using System.Threading.Tasks;

namespace Server.Services
{

    public interface ICertificateValidator
    {

        Task<bool> Validate(X509Certificate2 certificate);

    }

}
