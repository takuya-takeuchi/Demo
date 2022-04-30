using System.Collections.Generic;
using WPF.PcapNet.Models.Interfaces;

namespace WPF.PcapNet.Services.Interfaces
{

    public interface IPacketCaptureService
    {

        IEnumerable<IPacketCaptureDeviceModel> GetAllDevices();

    }

}
