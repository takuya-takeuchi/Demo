using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Claims;
using System.Security.Cryptography.X509Certificates;
using System.Threading.Tasks;

using Microsoft.AspNetCore.Authentication.Certificate;

using Demo.Services;

namespace Demo.Services
{

    public class CertificateValidationService : ICertificateValidationService
    {
        
        public bool IsValid(X509Certificate2 certificate)
        {
            bool isValid = true;

            // Custom validation logic

            return isValid;
        }

    }

}