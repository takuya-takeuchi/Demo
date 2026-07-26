using System.Net.Http.Headers;

namespace Demo
{

    public partial class ApiClient
    {

        #region Properties

        public string? AccessToken { get; set; }

        #endregion

        #region Methods

        partial void PrepareRequest(HttpClient client, HttpRequestMessage request, string url)
        {
            if (string.IsNullOrWhiteSpace(this.AccessToken))
                return;

            request.Headers.Authorization = new AuthenticationHeaderValue("Bearer", this.AccessToken);
        }

        #endregion

    }

}