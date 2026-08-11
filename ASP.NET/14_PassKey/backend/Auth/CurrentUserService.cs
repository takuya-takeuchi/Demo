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

        /// <summary>LastSeenAt を書き戻す間隔。毎リクエスト UPDATE すると SQLite の書き込みが無駄に増える。</summary>
        private static readonly TimeSpan LastSeenPrecision = TimeSpan.FromMinutes(5);

        /// <summary>
        /// ログイン直後はフロントから複数の API 呼び出しが同時に飛ぶため、
        /// 同じユーザーの初回 INSERT が競合する。ユーザー単位で直列化して衝突を防ぐ。
        /// （プロセスをまたぐ競合は下の DbUpdateException 側で吸収する）
        /// </summary>
        private static readonly ConcurrentDictionary<string, SemaphoreSlim> ProvisionLocks = new();

        #endregion

        #region Properties

        private ClaimsPrincipal User =>
            accessor.HttpContext?.User ?? throw new InvalidOperationException("HttpContext がありません。");

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
                    // ログイン直後は複数の API 呼び出しが同時に走るため、
                    // 初回作成の INSERT が衝突しうる（UNIQUE constraint failed: Users.Id）。
                    // 先に挿入された行を読み直して続行する。
                    db.Entry(user).State = EntityState.Detached;
                    user = await db.Users.FirstAsync(u => u.Id == id, ct);
                }
            }

            var dirty = false;
            if (email is not null && user.Email != email)
            {
                // Hanko 側でメールアドレスが変わった場合に追従する
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
