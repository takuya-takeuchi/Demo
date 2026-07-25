using System;

namespace Demo.Models
{

    public sealed record RefreshTokenEntry(string UserId,
                                           DateTimeOffset ExpiresAt);

}