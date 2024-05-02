using System.Security.Cryptography.X509Certificates;
using System.Threading.Tasks;

namespace Server.Services
{

    public sealed class IssuerCertificateValidator : ICertificateValidator
    {

        #region Fields

        private readonly string _Issuer;

        #endregion

        #region Constructors

        public IssuerCertificateValidator(string issuer)
        {
            this._Issuer = issuer;
        }

        #endregion

        #region Methods
        
        public async Task<bool> Validate(X509Certificate2 certificate)
        {
            return certificate.Issuer == this._Issuer;
        }

        #endregion

    }

}
