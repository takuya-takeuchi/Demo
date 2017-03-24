using System.Collections.ObjectModel;
using GalaSoft.MvvmLight.Command;
using WPF.PacketCapture.Models.Interfaces;

namespace WPF.PacketCapture.ViewModels.Interfaces
{

    public interface IMainViewModel
    {

        #region Properties

        ObservableCollection<IPacketCaptureDeviceModel> CaptureDevices
        {
            get;
        }

        ObservableCollection<IPacketModel> Packets
        {
            get;
        }

        IPacketCaptureDeviceModel SelectedCaptureDevice
        {
            get;
            set;
        }

        RelayCommand<IPacketCaptureDeviceModel> StartCaptureCommand
        {
            get;
        }

        RelayCommand<IPacketCaptureDeviceModel> StopCaptureCommand
        {
            get;
        }

        #endregion

    }

}