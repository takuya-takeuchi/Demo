using System;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Threading.Tasks;

using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.Logging;

namespace Demo
{

    internal sealed class HttpDumpMiddleware
    {

        #region Fields

        private const int MaxBodySize = 64 * 1024;

        private readonly RequestDelegate _Next;

        private readonly ILogger<HttpDumpMiddleware> _Logger;

        #endregion

        #region Constructors

        public HttpDumpMiddleware(
            RequestDelegate next,
            ILogger<HttpDumpMiddleware> logger)
        {
            _Next = next;
            _Logger = logger;
        }

        #endregion

        #region Methods

        public async Task InvokeAsync(HttpContext context)
        {
            var stopwatch = Stopwatch.StartNew();

            string requestDump = await DumpRequestAsync(context.Request);

            Stream originalResponseBody = context.Response.Body;

            await using var capturedResponseBody = new MemoryStream();
            context.Response.Body = capturedResponseBody;

            Exception? exception = null;

            try
            {
                await _Next(context);
            }
            catch (Exception ex)
            {
                exception = ex;
                throw;
            }
            finally
            {
                stopwatch.Stop();

                context.Response.Body = originalResponseBody;

                string responseDump =
                    await DumpResponseAsync(context.Response, capturedResponseBody);

                _Logger.LogInformation(
                    """
                    HTTP DUMP
                    ===== REQUEST =====
                    {Request}

                    ===== RESPONSE =====
                    {Response}

                    Duration: {ElapsedMilliseconds} ms
                    Exception: {Exception}
                    """,
                    requestDump,
                    responseDump,
                    stopwatch.Elapsed.TotalMilliseconds,
                    exception?.ToString() ?? "(none)");

                capturedResponseBody.Position = 0;
                await capturedResponseBody.CopyToAsync(
                    originalResponseBody,
                    context.RequestAborted);
            }
        }

        #region Helpers

        private static async Task<string> DumpRequestAsync(HttpRequest request)
        {
            var builder = new StringBuilder();

            builder.Append(request.Method);
            builder.Append(' ');
            builder.Append(request.PathBase);
            builder.Append(request.Path);
            builder.Append(request.QueryString);
            builder.Append(' ');
            builder.AppendLine(request.Protocol);

            foreach (var header in request.Headers)
            {
                builder.Append(header.Key);
                builder.Append(": ");
                builder.AppendLine(RedactHeader(header.Key, header.Value.ToString()));
            }

            if (!ShouldDumpBody(request.ContentType, request.ContentLength))
                return builder.ToString();

            request.EnableBuffering();
            request.Body.Position = 0;

            using var reader = new StreamReader(request.Body,
                                                Encoding.UTF8,
                                                detectEncodingFromByteOrderMarks: true,
                                                bufferSize: 4096,
                                                leaveOpen: true);

            var body = new char[MaxBodySize];
            int read = await reader.ReadBlockAsync(body, 0, body.Length);

            request.Body.Position = 0;

            builder.AppendLine();
            builder.Append(body, 0, read);

            if (request.ContentLength > MaxBodySize)
            {
                builder.AppendLine();
                builder.Append($"... truncated at {MaxBodySize} bytes");
            }

            return builder.ToString();
        }

        private static async Task<string> DumpResponseAsync(HttpResponse response, MemoryStream bodyStream)
        {
            var builder = new StringBuilder();

            builder.Append(response.HttpContext.Request.Protocol);
            builder.Append(' ');
            builder.AppendLine(response.StatusCode.ToString());

            foreach (var header in response.Headers)
            {
                builder.Append(header.Key);
                builder.Append(": ");
                builder.AppendLine(RedactHeader(header.Key, header.Value.ToString()));
            }

            if (!ShouldDumpBody(response.ContentType, bodyStream.Length))
                return builder.ToString();

            bodyStream.Position = 0;

            using var reader = new StreamReader(bodyStream,
                                                Encoding.UTF8,
                                                detectEncodingFromByteOrderMarks: true,
                                                bufferSize: 4096,
                                                leaveOpen: true);

            var body = new char[MaxBodySize];
            int read = await reader.ReadBlockAsync(body, 0, body.Length);

            bodyStream.Position = 0;

            builder.AppendLine();
            builder.Append(body, 0, read);

            if (bodyStream.Length > MaxBodySize)
            {
                builder.AppendLine();
                builder.Append($"... truncated at {MaxBodySize} bytes");
            }

            return builder.ToString();
        }

        private static string RedactHeader(string name, string value)
        {
            if (name.Equals("Authorization", StringComparison.OrdinalIgnoreCase) ||
                name.Equals("Cookie", StringComparison.OrdinalIgnoreCase) ||
                name.Equals("Set-Cookie", StringComparison.OrdinalIgnoreCase) ||
                name.Equals("X-Api-Key", StringComparison.OrdinalIgnoreCase))
                return "[REDACTED]";

            return value;
        }

        private static bool ShouldDumpBody(string? contentType, long? contentLength)
        {
            if (contentLength is 0)
                return false;

            if (string.IsNullOrWhiteSpace(contentType))
                return false;

            return contentType.StartsWith("application/json", StringComparison.OrdinalIgnoreCase) ||
                   contentType.StartsWith("application/problem+json", StringComparison.OrdinalIgnoreCase) ||
                   contentType.StartsWith("text/", StringComparison.OrdinalIgnoreCase) ||
                   contentType.StartsWith("application/xml", StringComparison.OrdinalIgnoreCase);
        }

        #endregion

        #endregion

    }

}