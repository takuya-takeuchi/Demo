using System;
using System.Runtime.Serialization;

namespace Server.Models
{

    [DataContract]
    [Serializable]
    public sealed class Greeting
    {

        [DataMember]
        public string Message
        {
            get;
            set;
        }

    }

}