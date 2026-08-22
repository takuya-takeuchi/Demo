using System;
using System.Threading.Tasks;

using Microsoft.EntityFrameworkCore;
using Microsoft.OpenApi;

using Demo.Auth;
using Demo.Data;

namespace Demo
{

    internal sealed class Program
    {

        #region Methods

        public static void Main(string[] args)
        {
            var builder = WebApplication.CreateBuilder(args);

            // LAN / スマホから HTTPS でアクセスするための設定を任意で読み込む。
            // scripts/setup-dev-https.(ps1|sh) がこのファイルを生成する。
            // 存在しなければ何もしないので、localhost だけで使う分には不要。
            builder.Configuration.AddJsonFile("appsettings.Local.json", optional: true, reloadOnChange: true);

            // SQLite
            builder.Services.AddDbContext<AppDbContext>(o =>
                o.UseSqlite(builder.Configuration.GetConnectionString("Default") ?? "Data Source=app.db"));

            // Hanko authentication (JWT verification via JWKS)
            builder.Services.AddHankoAuthentication(builder.Configuration);

            builder.Services.AddHttpContextAccessor();
            builder.Services.AddScoped<CurrentUserService>();

            // Reverse proxy to Hanko
            // The browser never talks to Hanko directly; it always goes through /auth/* here.
            // That leaves the browser with a single visible origin, which removes the need for CORS.
            // Routes and destinations are defined in the ReverseProxy section of appsettings.json.
            builder.Services.AddReverseProxy().LoadFromConfig(builder.Configuration.GetSection("ReverseProxy"));
            builder.Services.AddControllers();
            builder.Services.AddOpenApi();
            builder.Services.AddProblemDetails();

            var app = builder.Build();

            // Create the schema on startup
            // EnsureCreated is fine for a sample; use dotnet ef migrations in production.
            using (var scope = app.Services.CreateScope())
            {
                var db = scope.ServiceProvider.GetRequiredService<AppDbContext>();
                db.Database.EnsureCreated();

                // SQLite's default (journal_mode=delete) makes reads and writes block each other.
                // WAL lets reads and writes proceed at the same time.
                db.Database.ExecuteSqlRaw("PRAGMA journal_mode=WAL;");
            }

            // Return JSON (ProblemDetails) for 500s as well
            app.UseExceptionHandler();

            if (app.Environment.IsDevelopment())
            {
                app.MapOpenApi();
            }

            // SPA static files
            // `npm run build` in frontend emits into wwwroot.
            app.UseDefaultFiles();
            app.UseStaticFiles();

            app.UseAuthentication();
            app.UseAuthorization();

            // Routing
            app.MapGet("/health", () => Results.Ok(new { status = "ok" })).AllowAnonymous();

            // Forward /auth/* to Hanko (PathRemovePrefix strips /auth before forwarding)
            app.MapReverseProxy();

            // /api/me and /api/todos live in Controllers/. Each controller carries [Authorize],
            // which is what the RequireAuthorization() on the old minimal-API group used to do.
            app.MapControllers();

            // Return 404 for API paths that do not exist.
            // Without this, MapFallbackToFile below would catch them and answer a mistyped
            // API path with index.html (200), which is confusing to debug.
            app.Map("/api/{**rest}", () => Results.NotFound()).AllowAnonymous();

            // Everything else goes to the SPA (for client-side routing)
            app.MapFallbackToFile("index.html");

            app.Run();
        }

        #endregion

    }

}