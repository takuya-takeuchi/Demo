namespace Demo
{

    public class AuthSettings
    {
        
        public string ClientId { get; set; } = "";

        public string TenantId { get; set; } = "";

        public string AuthorityInstance { get; set; } = "https://login.microsoftonline.com/";

        public string RedirectUri { get; set; } = "http://localhost";

        public string[] Scopes { get; set; } = Array.Empty<string>();

    }

}