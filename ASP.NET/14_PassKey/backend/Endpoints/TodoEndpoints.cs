using Microsoft.EntityFrameworkCore;

using Demo.Auth;
using Demo.Data;
using Demo.Models;

namespace Demo.Endpoints
{

    public static class TodoEndpoints
    {

        #region Methods

        public static IEndpointRouteBuilder MapAppEndpoints(this IEndpointRouteBuilder app)
        {
            var api = app.MapGroup("/api").RequireAuthorization();

            api.MapGet("/me", async (CurrentUserService current, CancellationToken ct) =>
            {
                var user = await current.GetOrCreateAsync(ct);
                return Results.Ok(new MeDto(user.Id, user.Email, user.DisplayName, user.CreatedAt));
            });

            var todos = api.MapGroup("/todos");

            todos.MapGet("/", async (AppDbContext db, CurrentUserService current, CancellationToken ct) =>
            {
                var user = await current.GetOrCreateAsync(ct);

                var items = await db.Todos
                    .Where(t => t.OwnerId == user.Id)
                    .OrderByDescending(t => t.CreatedAt)
                    .Select(t => new TodoDto(t.Id, t.Title, t.IsDone, t.CreatedAt))
                    .ToListAsync(ct);

                return Results.Ok(items);
            });

            todos.MapPost("/", async (CreateTodoRequest body, AppDbContext db, CurrentUserService current, CancellationToken ct) =>
            {
                if (string.IsNullOrWhiteSpace(body.Title))
                {
                    return Results.ValidationProblem(new Dictionary<string, string[]>
                    {
                        ["title"] = ["タイトルは必須です。"]
                    });
                }

                var user = await current.GetOrCreateAsync(ct);

                var todo = new TodoItem { Title = body.Title.Trim(), OwnerId = user.Id };
                db.Todos.Add(todo);
                await db.SaveChangesAsync(ct);

                return Results.Created($"/api/todos/{todo.Id}",
                    new TodoDto(todo.Id, todo.Title, todo.IsDone, todo.CreatedAt));
            });

            todos.MapPatch("/{id:int}", async (
                int id, UpdateTodoRequest body, AppDbContext db, CurrentUserService current, CancellationToken ct) =>
            {
                var userId = current.UserId;

                // Putting the ownership check in the WHERE clause makes someone else's todo look nonexistent
                var todo = await db.Todos.FirstOrDefaultAsync(t => t.Id == id && t.OwnerId == userId, ct);
                if (todo is null) return Results.NotFound();

                if (body.Title is not null) todo.Title = body.Title.Trim();
                if (body.IsDone is not null) todo.IsDone = body.IsDone.Value;

                await db.SaveChangesAsync(ct);
                return Results.Ok(new TodoDto(todo.Id, todo.Title, todo.IsDone, todo.CreatedAt));
            });

            todos.MapDelete("/{id:int}", async (
                int id, AppDbContext db, CurrentUserService current, CancellationToken ct) =>
            {
                var userId = current.UserId;

                var deleted = await db.Todos
                    .Where(t => t.Id == id && t.OwnerId == userId)
                    .ExecuteDeleteAsync(ct);

                return deleted == 0 ? Results.NotFound() : Results.NoContent();
            });

            return app;
        }

        #endregion

    }

}
