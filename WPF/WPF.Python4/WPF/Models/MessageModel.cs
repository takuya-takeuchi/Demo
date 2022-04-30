using Newtonsoft.Json;

namespace WPFPython.Models
{
    
    [JsonObject("message")]
    public sealed class MessageModel
    {

        [JsonProperty("hello")]
        public string Hello { get; set; }

    }
}
