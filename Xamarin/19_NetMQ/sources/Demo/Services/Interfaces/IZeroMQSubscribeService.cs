using System;

namespace Demo.Services.Interfaces
{

    public interface IZeroMQSubscribeService
    {
        
        event EventHandler<bool> StateChanged;

        void Connect(string address, Action<string> callback);

        void Disconnect();

    }

}