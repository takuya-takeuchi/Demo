using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;

using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.EntityFrameworkCore;

using Demo.Auth;
using Demo.Data;
using Demo.Models;

namespace Demo.Controllers
{

    /// <summary>
    /// Todo items. Every action is scoped to the signed-in user.
    /// </summary>
    [ApiController]
    [Authorize]
    // Spelled out instead of "api/[controller]": that token keeps the class casing, which would
    // publish the route as /api/Todos and show up capitalised in the OpenAPI document.
    [Route("api/todos")]
    [Produces("application/json")]
    public sealed class TodosController(AppDbContext db, CurrentUserService currentUser) : ControllerBase
    {

        #region Methods

        /// <summary>
        /// Lists the current user's todos, newest first.
        /// </summary>
        [HttpGet]
        [ProducesResponseType(typeof(IEnumerable<TodoDto>), StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status401Unauthorized)]
        public async Task<ActionResult<IEnumerable<TodoDto>>> List(CancellationToken ct)
        {
            var user = await currentUser.GetOrCreateAsync(ct);

            var items = await db.Todos.Where(t => t.OwnerId == user.Id)
                                      .OrderByDescending(t => t.CreatedAt)
                                      .Select(t => new TodoDto(t.Id, t.Title, t.IsDone, t.CreatedAt))
                                      .ToListAsync(ct);

            return this.Ok(items);
        }

        /// <summary>
        /// Creates a todo owned by the current user.
        /// </summary>
        [HttpPost]
        [ProducesResponseType(typeof(TodoDto), StatusCodes.Status201Created)]
        [ProducesResponseType(StatusCodes.Status400BadRequest)]
        [ProducesResponseType(StatusCodes.Status401Unauthorized)]
        public async Task<ActionResult<TodoDto>> Create(CreateTodoRequest request, CancellationToken ct)
        {
            if (string.IsNullOrWhiteSpace(request.Title))
            {
                this.ModelState.AddModelError("title", "タイトルは必須です。");
                return this.ValidationProblem(this.ModelState);
            }

            var user = await currentUser.GetOrCreateAsync(ct);

            var todo = new TodoItem { Title = request.Title.Trim(), OwnerId = user.Id };
            db.Todos.Add(todo);
            await db.SaveChangesAsync(ct);

            return this.Created($"/api/todos/{todo.Id}", new TodoDto(todo.Id, todo.Title, todo.IsDone, todo.CreatedAt));
        }

        /// <summary>
        /// Updates the title and/or the done flag of one todo.
        /// </summary>
        [HttpPatch("{id:int}")]
        [ProducesResponseType(typeof(TodoDto), StatusCodes.Status200OK)]
        [ProducesResponseType(StatusCodes.Status401Unauthorized)]
        [ProducesResponseType(StatusCodes.Status404NotFound)]
        public async Task<ActionResult<TodoDto>> Update(int id, UpdateTodoRequest request, CancellationToken ct)
        {
            var userId = currentUser.UserId;

            // Putting the ownership check in the WHERE clause makes someone else's todo look nonexistent
            var todo = await db.Todos.FirstOrDefaultAsync(t => t.Id == id && t.OwnerId == userId, ct);
            if (todo is null)
            {
                return this.NotFound();
            }

            if (request.Title is not null) todo.Title = request.Title.Trim();
            if (request.IsDone is not null) todo.IsDone = request.IsDone.Value;

            await db.SaveChangesAsync(ct);

            return this.Ok(new TodoDto(todo.Id, todo.Title, todo.IsDone, todo.CreatedAt));
        }

        /// <summary>
        /// Deletes one todo.
        /// </summary>
        [HttpDelete("{id:int}")]
        [ProducesResponseType(StatusCodes.Status204NoContent)]
        [ProducesResponseType(StatusCodes.Status401Unauthorized)]
        [ProducesResponseType(StatusCodes.Status404NotFound)]
        public async Task<IActionResult> Delete(int id, CancellationToken ct)
        {
            var userId = currentUser.UserId;

            var deleted = await db.Todos.Where(t => t.Id == id && t.OwnerId == userId)
                                        .ExecuteDeleteAsync(ct);

            return deleted == 0 ? this.NotFound() : this.NoContent();
        }

        #endregion

    }

}
