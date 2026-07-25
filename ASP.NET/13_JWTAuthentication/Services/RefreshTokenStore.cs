using System;
using System.Collections.Concurrent;

using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.Logging;

using Demo.Models;
using Demo.Options;

namespace Demo.Services
{

    public sealed class RefreshTokenStore
    {

        #region Fields

        private readonly ConcurrentDictionary<string, RefreshTokenEntry> _Entries = new(StringComparer.Ordinal);

        #endregion

        #region Methods

        public void Store(string refreshToken,
                          string userId,
                          DateTimeOffset expiresAt)
        {
            _Entries[refreshToken] = new RefreshTokenEntry(userId, expiresAt);
        }

        public RefreshTokenEntry? Find(string refreshToken)
        {
            return _Entries.TryGetValue(refreshToken, out RefreshTokenEntry? entry)
                ? entry
                : null;
        }

        public bool Remove(string refreshToken)
        {
            return _Entries.TryRemove(refreshToken, out _);
        }

        public bool TryConsume(string refreshToken, out RefreshTokenEntry? entry)
        {
            return _Entries.TryRemove(refreshToken, out entry);
        }

        #endregion

    }

}