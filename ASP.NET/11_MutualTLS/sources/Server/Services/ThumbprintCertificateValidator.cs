using System;
using System.Security.Cryptography.X509Certificates;
using System.Threading.Tasks;

namespace Server.Services
{

    public sealed class ThumbprintCertificateValidator : ICertificateValidator
    {

        #region Fields

        private readonly string _Thumbprint;

        #endregion

        #region Constructors

        public ThumbprintCertificateValidator(string thumbprint)
        {
            this._Thumbprint = thumbprint;
        }

        #endregion

        #region Methods
        
        public async Task<bool> Validate(X509Certificate2 certificate)
        {
            return string.Equals(certificate.Thumbprint, this._Thumbprint, StringComparison.CurrentCultureIgnoreCase);
        }

        #endregion

    }

}
