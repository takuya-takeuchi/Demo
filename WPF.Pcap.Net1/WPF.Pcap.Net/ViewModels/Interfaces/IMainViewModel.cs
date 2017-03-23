using System.Collections.ObjectModel;
using GalaSoft.MvvmLight.Command;
using WPF.PcapNet.Models.Interfaces;

namespace WPF.PcapNet.ViewModels.Interfaces
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