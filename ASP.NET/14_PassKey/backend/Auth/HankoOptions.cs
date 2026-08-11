namespace Demo.Auth
{

    public class HankoOptions
    {

        #region Properties

        public const string SectionName = "Hanko";

        /// <summary>
        /// URL used by this server to reach Hanko. Used for fetching the JWKS and validating sessions.
        /// The browser never calls it directly; it always goes through /auth (the YARP proxy), so Hanko
        /// only needs to be exposed on the internal network.
        /// When running inside docker compose, point this at the service name (http://hanko:8000).
        /// </summary>
        public string InternalApiUrl { get; set; } = "http://localhost:8000";

        /// <summary>Name of the cookie carrying the session token. Hanko's default is "hanko".</summary>
        public string CookieName { get; set; } = "hanko";

        /// <summary>
        /// Whether to validate the aud claim. Hanko Cloud puts the App URL in aud, so true is recommended there.
        /// When self-hosting, leave it false unless session.audience is set in config.yaml.
        /// </summary>
        public bool ValidateAudience { get; set; }

        public string[] ValidAudiences { get; set; } = [];

        /// <summary>
        /// When true, calls Hanko's /sessions/validate in addition to verifying the JWT signature, so that
        /// server-side revocation is honoured (i.e. logout takes effect immediately).
        /// </summary>
        public bool ValidateSessionRemotely { get; set; }

        #endregion

    }

}
