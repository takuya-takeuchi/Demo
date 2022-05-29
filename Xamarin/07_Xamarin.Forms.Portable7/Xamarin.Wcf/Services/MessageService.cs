using System;
using System.Runtime.Serialization;
using System.ServiceModel;

namespace Xamarin.Wcf.Services
{
    
    [System.ServiceModel.ServiceContract]
    public interface IMessageService
    {

        event EventHandler<MessageDto> ReceivedImage;

        [System.ServiceModel.OperationContract]
        void SendMessage(MessageDto message);

    }

    [ServiceBehavior(InstanceContextMode = InstanceContextMode.Single)]
    public sealed class MessageService : IMessageService
    {

        public event EventHandler<MessageDto> ReceivedImage;


        public void SendMessage(MessageDto message)
        {
            this.ReceivedImage?.Invoke(this, message);
        }

    }


    [DataContract(Name = "messageDto")]
    public sealed class MessageDto
    {

        public MessageDto(string sender, string message)
        {
            this.Sender = sender;
            this.Message = message;
        }

        [DataMember(Name = "sender")]
        public string Sender
        {
            get;
            set;
        }

        [DataMember(Name = "message")]
        public string Message
        {
            get;
            set;
        }
    }

}
