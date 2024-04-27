using System;
using System.Text.Json.Serialization;

namespace Demo.Models
{

    /// <summary>
    /// Message.
    /// </summary>
    public sealed class Message
    {

        #region Properties

        /// <summary>
        /// Get or set date.
        /// </summary>
        [JsonPropertyName("date")]
        public DateTime Date
        {
            get;
            set;
        }

        /// <summary>
        /// Get or set text.
        /// </summary>
        [JsonPropertyName("text")]
        public string Text
        {
            get;
            set;
        }

        #endregion

    }

}
