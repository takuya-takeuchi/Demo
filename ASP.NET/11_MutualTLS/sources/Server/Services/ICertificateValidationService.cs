using System.Security.Cryptography.X509Certificates;

namespace Demo.Services
{

    public interface ICertificateValidationService
    {
        
        bool IsValid(X509Certificate2 certificate);

    }

}