using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

using Microsoft.AspNetCore.Authentication;
using Microsoft.AspNetCore.Authentication.Certificate;
using Microsoft.Extensions.DependencyInjection;

using Demo.Services;

namespace Demo.Extensions
{
    public static class AddCertificateAuthenticationExtension
    {
        public static IServiceCollection AddCertificateAuthentication(this IServiceCollection services)
        {
            services.AddTransient<ICertificateValidationService, CertificateValidationService>();
            services.AddAuthentication(CertificateAuthenticationDefaults.AuthenticationScheme)
                .AddCertificate( options =>
                {
                    options.AllowedCertificateTypes = CertificateTypes.All;

                    options.Events = new CertificateAuthenticationEvents
                    {
                        OnCertificateValidated = context =>
                        {
                            var validationService = context.HttpContext.RequestServices
                            .GetRequiredService<ICertificateValidationService>();

                            if (!validationService.IsValid(context.ClientCertificate))
                            {
                                context.Fail("Invalid certificate.");
                                return Task.CompletedTask;
                            }

                            context.Success();
                            return Task.CompletedTask;
                        }
                    };
                })
                .AddCertificateCache();

            return services;
        }
    }
}