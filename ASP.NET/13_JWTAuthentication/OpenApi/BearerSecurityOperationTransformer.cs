using System;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.OpenApi;
using Microsoft.OpenApi;

namespace Demo.OpenApi
{

    internal sealed class BearerSecurityOperationTransformer : IOpenApiOperationTransformer
    {

        #region Methods

        public Task TransformAsync(OpenApiOperation operation,
                                   OpenApiOperationTransformerContext context,
                                   CancellationToken cancellationToken)
        {
            object[] metadata = context.Description
                                       .ActionDescriptor
                                       .EndpointMetadata
                                       .ToArray();

            bool hasAllowAnonymous = metadata.OfType<IAllowAnonymous>().Any();
            if (hasAllowAnonymous)
                return Task.CompletedTask;

            bool hasAuthorize = metadata.OfType<IAuthorizeData>().Any();
            if (!hasAuthorize)
                return Task.CompletedTask;

            operation.Security ??= [];

            operation.Security.Add(new OpenApiSecurityRequirement
            {
                [new OpenApiSecuritySchemeReference("Bearer", context.Document)] = []
            });

            return Task.CompletedTask;
        }

        #endregion

    }

}