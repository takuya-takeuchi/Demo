using System.Collections.Concurrent;
using System.Security.Claims;

using Microsoft.EntityFrameworkCore;

using Demo.Data;
using Demo.Models;

namespace Demo.Auth
{

    public class CurrentUserService(AppDbContext db, IHttpContextAccessor accessor)
    {

        #region Fields

        /// <summary>How often LastSeenAt is written back. Updating it on every request would waste SQLite writes.</summary>
        private static readonly TimeSpan LastSeenPrecision = TimeSpan.FromMinutes(5);

        /// <summary>
        /// Right after login the frontend fires several API calls at once, so the first INSERT for the
        /// same user can race. Serialize per user to avoid the collision.
        /// (Races across processes are absorbed by the DbUpdateException handler below.)
        /// </summary>
        private static readonly ConcurrentDictionary<string, SemaphoreSlim> ProvisionLocks = new();

        #endregion

        #region Properties

        private ClaimsPrincipal User =>
            accessor.HttpContext?.User ?? throw new InvalidOperationException("No HttpContext is available.");

        public string UserId => User.GetHankoUserId();

        #endregion

        #region Methods

        public async Task<AppUser> GetOrCreateAsync(CancellationToken ct = default)
        {
            var gate = ProvisionLocks.GetOrAdd(UserId, _ => new SemaphoreSlim(1, 1));
            await gate.WaitAsync(ct);

            try
            {
                return await GetOrCreateCoreAsync(ct);
            }
            finally
            {
                gate.Release();
            }
        }

        #region Helpers

        private async Task<AppUser> GetOrCreateCoreAsync(CancellationToken ct)
        {
            var id = UserId;
            var email = User.GetEmail();

            var user = await db.Users.FirstOrDefaultAsync(u => u.Id == id, ct);

            if (user is null)
            {
                user = new AppUser
                {
                    Id = id,
                    Email = email,
                    DisplayName = email?.Split('@')[0],
                };
                db.Users.Add(user);

                try
                {
                    await db.SaveChangesAsync(ct);
                    return user;
                }
                catch (DbUpdateException)
                {
                    // Several API calls run concurrently right after login, so the INSERT that
                    // provisions the user can collide (UNIQUE constraint failed: Users.Id).
                    // Re-read the row that won the race and carry on.
                    db.Entry(user).State = EntityState.Detached;
                    user = await db.Users.FirstAsync(u => u.Id == id, ct);
                }
            }

            var dirty = false;
            if (email is not null && user.Email != email)
            {
                // Follow along when the email address is changed on the Hanko side
                user.Email = email;
                dirty = true;
            }

            var now = DateTime.UtcNow;
            if (now - user.LastSeenAt > LastSeenPrecision)
            {
                user.LastSeenAt = now;
                dirty = true;
            }

            if (dirty)
            {
                await db.SaveChangesAsync(ct);
            }

            return user;
        }

        #endregion

        #endregion

    }

}
