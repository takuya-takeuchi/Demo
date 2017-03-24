using System.Collections.Generic;
using WPF.PacketCapture.Models.Interfaces;

namespace WPF.PacketCapture.Services.Interfaces
{

    public interface IPacketCaptureService
    {

        IEnumerable<IPacketCaptureDeviceModel> GetAllDevices();

    }

}
