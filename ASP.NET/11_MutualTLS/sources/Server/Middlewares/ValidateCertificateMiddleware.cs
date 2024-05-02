using Server.Services;
using System.Threading.Tasks;

using Microsoft.AspNetCore.Http;

namespace Server.Middlewares
{

    public class ValidateCertificateMiddleware
    {

        #region Fields

        private readonly RequestDelegate _Next;

        private readonly ICertificateValidator _Validator;

        #endregion

        #region Constructors

        public ValidateCertificateMiddleware(RequestDelegate next, ICertificateValidator validator)
        {
            this._Next = next;
            this._Validator = validator;
        }

        #endregion

        #region Methods

        public async Task Invoke(HttpContext context)
        {
            var certificate = context.Connection.ClientCertificate;
            if (certificate != null && await this._Validator.Validate(certificate))
            {
                await _Next(context);
            }
            else
            {
                context.Response.StatusCode = 403;
                await context.Response.WriteAsync("Invalid client certificate.");
            }
        }

        #endregion

    }

}