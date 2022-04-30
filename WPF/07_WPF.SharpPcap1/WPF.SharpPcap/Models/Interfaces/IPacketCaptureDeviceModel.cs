using System;

namespace WPF.PacketCapture.Models.Interfaces
{

    public interface IPacketCaptureDeviceModel
    {

        #region Events

        event EventHandler<IPacketModel> PacketReceived;

        #endregion

        #region Properties

        string Description
        {
            get;
        }

        string Name
        {
            get;
        }

        #endregion

        #region Methods

        void StartCapture();

        void StopCapture();

        #endregion

    }

}