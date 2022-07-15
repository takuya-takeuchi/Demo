using System;

namespace Demo.Services.Interfaces
{

    internal interface IZeroMQSubscribeService
    {

        void Connect(string address, Action<string> callback);

        void Disconnect();

    }

}