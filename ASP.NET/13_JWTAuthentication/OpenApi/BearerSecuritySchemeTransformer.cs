using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

using Microsoft.AspNetCore.Authentication;
using Microsoft.AspNetCore.OpenApi;
using Microsoft.IdentityModel.Tokens;
using Microsoft.OpenApi;

namespace Demo.OpenApi
{

    internal sealed class BearerSecuritySchemeTransformer : IOpenApiDocumentTransformer
    {

        #region Fields

        private readonly IAuthenticationSchemeProvider _AuthenticationSchemeProvider;

        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a new instance of the <see cref="BearerSecuritySchemeTransformer"/> class with the specified logger.
        /// </summary>
        public BearerSecuritySchemeTransformer(IAuthenticationSchemeProvider authenticationSchemeProvider)
        {
            this._AuthenticationSchemeProvider = authenticationSchemeProvider;
        }

        #endregion

        #region Methods

        public async Task TransformAsync(OpenApiDocument document,
                                         OpenApiDocumentTransformerContext context,
                                         CancellationToken cancellationToken)
        {
            var authenticationSchemes = await this._AuthenticationSchemeProvider.GetAllSchemesAsync();
            bool hasBearer = authenticationSchemes.Any(scheme => scheme.Name == "Bearer");
            if (!hasBearer)
                return;

            document.Components ??= new OpenApiComponents();

            document.Components.SecuritySchemes = new Dictionary<string, IOpenApiSecurityScheme>
            {
                ["Bearer"] = new OpenApiSecurityScheme
                {
                    Type = SecuritySchemeType.Http,
                    Scheme = "bearer",
                    BearerFormat = "JWT",
                    In = ParameterLocation.Header,
                    Description = "Enter the JWT access token. Do not include the 'Bearer ' prefix."
                }
            };

            foreach (var path in document.Paths.Values)
            {
                foreach (var operation in path.Operations.Values)
                {
                    operation.Security ??= [];
                    operation.Security.Add(new OpenApiSecurityRequirement
                    {
                        [new OpenApiSecuritySchemeReference(
                            "Bearer",
                            document)] = []
                    });
                }
            }
        }

        #endregion

    }

}