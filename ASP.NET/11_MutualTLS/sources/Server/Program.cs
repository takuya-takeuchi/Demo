using System;
using System.IO;
using System.Net.Security;
using System.Reflection;
using System.Security.Authentication;

using Microsoft.AspNetCore.Authentication.Certificate;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Server.Kestrel.Https;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;
using Microsoft.OpenApi.Models;

using Server.Middlewares;
using Server.Services;

namespace Server
{

    internal sealed class Program
    {

        #region Methods

        public static void Main(string[] args)
        {
            var builder = WebApplication.CreateBuilder(args);
            builder.Logging.ClearProviders();
            builder.Logging.AddConsole();
            builder.Services.AddControllers();
            builder.Services.AddEndpointsApiExplorer();
            builder.Services.AddSwaggerGen(options =>
            {
                options.SwaggerDoc("v1", new OpenApiInfo
                {
                    Version = "v1",
                    Title = "ToDo API",
                    Description = "An ASP.NET Core Web API for managing ToDo items",
                    TermsOfService = new Uri("https://example.com/terms"),
                    Contact = new OpenApiContact
                    {
                        Name = "Example Contact",
                        Url = new Uri("https://example.com/contact")
                    },
                    License = new OpenApiLicense
                    {
                        Name = "Example License",
                        Url = new Uri("https://example.com/license")
                    }
                });

                // using System.Reflection;
                var xmlFilename = $"{Assembly.GetExecutingAssembly().GetName().Name}.xml";
                options.IncludeXmlComments(Path.Combine(AppContext.BaseDirectory, xmlFilename));
            });

            // for mTLS
            builder.WebHost.ConfigureKestrel(serverOptions =>
            {
                serverOptions.ConfigureHttpsDefaults(httpsOptions =>
                {
                    httpsOptions.ClientCertificateMode = ClientCertificateMode.RequireCertificate;
                    // You can set true but chainStatus could have RevocationStatusUnknown
                    httpsOptions.CheckCertificateRevocation = false;
                    httpsOptions.SslProtocols = SslProtocols.Tls12 | SslProtocols.Tls13;
                    httpsOptions.ClientCertificateValidation = (certificate, chain, errors) =>
                    {
                        // Validation will be executed in ValidateCertificateMiddleware
                        // This statement is mandatory and must return true
                        // HttpContext.Connection.ClientCertificate is always null in ValidateCertificateMiddleware.Validate if this statement is missing

                        if (errors == SslPolicyErrors.None)
                            return true;

                        // Do check chain.ChainStatus.
                        // NotValidForUsage: Check whether client authentication has flag clientAuth in extendedKeyUsage
                        if (errors == SslPolicyErrors.RemoteCertificateChainErrors)
                            return true;

                        return false;
                    };
                });
            });
            builder.Services.AddAuthentication(CertificateAuthenticationDefaults.AuthenticationScheme).AddCertificate();
            builder.Services.AddTransient<ICertificateValidator>(provider => new ThumbprintCertificateValidator("d917a6c41d5623c48b92358b67bc301c9f915254"));

            var app = builder.Build();

            // for mTLS
            app.UseMiddleware<ValidateCertificateMiddleware>();

            if (app.Environment.IsDevelopment())
            {
                app.UseSwagger();
                app.UseSwaggerUI(options =>
                {
                    options.SwaggerEndpoint("/swagger/v1/swagger.json", "v1");
                    options.RoutePrefix = string.Empty;
                });
            }

            app.UseHttpsRedirection();
            app.UseAuthorization();
            app.MapControllers();
            app.Run();
        }

        #endregion

    }

}