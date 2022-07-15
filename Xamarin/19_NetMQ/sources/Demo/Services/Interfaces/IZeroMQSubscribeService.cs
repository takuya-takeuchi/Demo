using System;

namespace Demo.Services.Interfaces
{

    public interface IZeroMQSubscribeService
    {

        void Connect(string address, Action<string> callback);

        void Disconnect();

    }

}