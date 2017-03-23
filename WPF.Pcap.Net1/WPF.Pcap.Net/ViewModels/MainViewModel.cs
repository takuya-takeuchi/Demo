using System;
using System.Collections.ObjectModel;
using System.Linq;
using System.Windows;
using System.Windows.Threading;
using GalaSoft.MvvmLight;
using GalaSoft.MvvmLight.Command;
using WPF.PcapNet.Models.Interfaces;
using WPF.PcapNet.Services.Interfaces;
using WPF.PcapNet.ViewModels.Interfaces;

namespace WPF.PcapNet.ViewModels
{

    internal sealed class MainViewModel : ViewModelBase, IMainViewModel
    {

        #region Fields

        private readonly IPacketCaptureService _PacketCaptureService;

        private IPacketCaptureDeviceModel _SelectedCaptureDevice;

        private RelayCommand<IPacketCaptureDeviceModel> _StartCaptureCommand;

        private RelayCommand<IPacketCaptureDeviceModel> _StopCaptureCommand;

        #endregion

        #region Constructors

        public MainViewModel(IPacketCaptureService packetCaptureService)
        {
            if (packetCaptureService == null)
                throw new ArgumentNullException(nameof(packetCaptureService));

            this._PacketCaptureService = packetCaptureService;

            this.CaptureDevices = new ObservableCollection<IPacketCaptureDeviceModel>(packetCaptureService.GetAllDevices());
            this.SelectedCaptureDevice = this.CaptureDevices.FirstOrDefault();
            this.Packets = new ObservableCollection<IPacketModel>();
        }

        #endregion

        #region Properties

        public ObservableCollection<IPacketCaptureDeviceModel> CaptureDevices
        {
            get;
        }

        public ObservableCollection<IPacketModel> Packets
        {
            get;
        }

        public IPacketCaptureDeviceModel SelectedCaptureDevice
        {
            get
            {
                return this._SelectedCaptureDevice;
            }
            set
            {
                // 選択が切り替わるなら切り替え前は停止
                var packetCaptureDeviceModel = this._SelectedCaptureDevice;
                if (packetCaptureDeviceModel != null)
                {
                    packetCaptureDeviceModel.StopCapture();
                    packetCaptureDeviceModel.PacketReceived -= this.SelectedCaptureDeviceOnPacketReceived;
                }

                this._SelectedCaptureDevice = value;
                this.StartCaptureCommand?.RaiseCanExecuteChanged();
                this.StopCaptureCommand?.RaiseCanExecuteChanged();
            }
        }

        public RelayCommand<IPacketCaptureDeviceModel> StartCaptureCommand
        {
            get
            {
                return this._StartCaptureCommand ?? (this._StartCaptureCommand = new RelayCommand<IPacketCaptureDeviceModel>(
                           model =>
                           {
                               this._SelectedCaptureDevice?.StopCapture();
                               this.Packets.Clear();
                               this._SelectedCaptureDevice.PacketReceived += this.SelectedCaptureDeviceOnPacketReceived;
                               this._SelectedCaptureDevice.StartCapture();
                           },
                           model => this._PacketCaptureService != null));
            }
        }

        public RelayCommand<IPacketCaptureDeviceModel> StopCaptureCommand
        {
            get
            {
                return this._StopCaptureCommand ?? (this._StopCaptureCommand = new RelayCommand<IPacketCaptureDeviceModel>(
                           model =>
                           {
                               this._SelectedCaptureDevice.StopCapture();
                               this._SelectedCaptureDevice.PacketReceived -= this.SelectedCaptureDeviceOnPacketReceived;
                           },
                           model => this._PacketCaptureService != null));

            }
        }

        #endregion

        #region Methods

        #region Helper Methods

        private void SelectedCaptureDeviceOnPacketReceived(object sender, IPacketModel packetModel)
        {
            Application.Current.Dispatcher.Invoke(() =>this.Packets.Insert(0, packetModel), DispatcherPriority.Background);
        }

        #endregion

        #endregion

    }

}