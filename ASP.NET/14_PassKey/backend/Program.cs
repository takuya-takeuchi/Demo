using System;
using System.Threading.Tasks;

using Microsoft.EntityFrameworkCore;
using Microsoft.OpenApi;

using Demo.Auth;
using Demo.Data;
using Demo.Endpoints;

namespace Demo
{

    internal sealed class Program
    {

        #region Methods

        public static void Main(string[] args)
        {
            var builder = WebApplication.CreateBuilder(args);

            // --- SQLite -----------------------------------------------------------
            builder.Services.AddDbContext<AppDbContext>(o =>
                o.UseSqlite(builder.Configuration.GetConnectionString("Default") ?? "Data Source=app.db"));

            // --- Hanko 認証（JWKS による JWT 検証）--------------------------------
            builder.Services.AddHankoAuthentication(builder.Configuration);

            builder.Services.AddHttpContextAccessor();
            builder.Services.AddScoped<CurrentUserService>();

            // --- Hanko へのリバースプロキシ ---------------------------------------
            // ブラウザは Hanko に直接アクセスせず、常に /auth/* 経由でここを通る。
            // これによりブラウザから見えるオリジンが 1 つになり、CORS が不要になる。
            // ルートと転送先は appsettings.json の ReverseProxy セクションで定義している。
            builder.Services.AddReverseProxy()
                .LoadFromConfig(builder.Configuration.GetSection("ReverseProxy"));

            builder.Services.AddOpenApi();
            builder.Services.AddProblemDetails();

            var app = builder.Build();

            // --- 起動時にスキーマを作成 -------------------------------------------
            // サンプルなので EnsureCreated。実運用では dotnet ef migrations を使う。
            using (var scope = app.Services.CreateScope())
            {
                var db = scope.ServiceProvider.GetRequiredService<AppDbContext>();
                db.Database.EnsureCreated();

                // SQLite の既定（journal_mode=delete）は読み書きが互いをブロックする。
                // WAL にすると読み込みと書き込みが同時に走れるようになる。
                db.Database.ExecuteSqlRaw("PRAGMA journal_mode=WAL;");
            }

            // 例外時も 500 の JSON（ProblemDetails）を返す
            app.UseExceptionHandler();

            if (app.Environment.IsDevelopment())
            {
                app.MapOpenApi();
            }

            // --- SPA の静的ファイル -----------------------------------------------
            // frontend の `npm run build` が wwwroot に出力する。
            app.UseDefaultFiles();
            app.UseStaticFiles();

            app.UseAuthentication();
            app.UseAuthorization();

            // --- ルーティング -----------------------------------------------------
            app.MapGet("/health", () => Results.Ok(new { status = "ok" })).AllowAnonymous();

            // /auth/* を Hanko に転送する（PathRemovePrefix で /auth を落としてから渡す）
            app.MapReverseProxy();

            app.MapAppEndpoints();

            // 存在しない API パスは 404 を返す。
            // これが無いと下の MapFallbackToFile が拾ってしまい、
            // API のタイプミスに対して index.html（200）が返るという分かりにくい挙動になる。
            app.Map("/api/{**rest}", () => Results.NotFound()).AllowAnonymous();

            // それ以外は SPA に渡す（クライアントサイドルーティング用）
            app.MapFallbackToFile("index.html");

            app.Run();
        }

        #endregion

    }

}