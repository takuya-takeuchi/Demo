using System;

using NetMQ;
using NetMQ.Sockets;

using Demo.Services.Interfaces;

namespace Demo.Services
{

    internal sealed class ZeroMQSubscribeService : IZeroMQSubscribeService
    {
        
        #region Fields

        private Action<string> _Callback;

        private NetMQPoller _NetMqPoller;

        private SubscriberSocket _SubscriberSocket;

        #endregion

        #region Methods

        public void Connect(string address, Action<string> callback)
        {
            if (this._SubscriberSocket != null)
                return;

            this._Callback = callback;

            this._SubscriberSocket = new SubscriberSocket();
            this._SubscriberSocket.ReceiveReady += this.ReceiveReady;
            this._SubscriberSocket.Connect(address);
            this._SubscriberSocket.SubscribeToAnyTopic();

            this._NetMqPoller = new NetMQPoller();
            this._NetMqPoller.Add(this._SubscriberSocket);
            this._NetMqPoller.RunAsync();
        }

        public void Disconnect()
        {
            if (this._SubscriberSocket == null)
                return;
            
            this._SubscriberSocket.ReceiveReady -= this.ReceiveReady;

            this._NetMqPoller.RemoveAndDispose(this._SubscriberSocket);
            this._NetMqPoller?.Dispose();

            this._NetMqPoller = null;
            this._SubscriberSocket = null;
        }

        #region Event Handlers

        private void ReceiveReady(object sender, NetMQSocketEventArgs e)
        {
            var messageReceived = e.Socket.ReceiveFrameString();
            this._Callback.Invoke(messageReceived);
        }

        #endregion

        #endregion

    }

}
