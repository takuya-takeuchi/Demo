using System.ComponentModel.DataAnnotations;
using System.Text.Json.Serialization;

namespace SwaggerApiDemo.Models
{

    /// <summary>
    /// Result.
    /// </summary>
    public sealed class Result
    {

        #region Properties
        
        /// <summary>
        /// Get or set status code of result.
        /// </summary>
        [JsonPropertyName("statusCode")]
        [Required]
        public int StatusCode
        {
            get;
            set;
        }

        #endregion

    }

}
