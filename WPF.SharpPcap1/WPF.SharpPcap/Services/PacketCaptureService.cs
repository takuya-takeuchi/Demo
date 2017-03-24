using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using PacketDotNet;
using SharpPcap;
using WPF.PacketCapture.Models.Interfaces;
using WPF.PacketCapture.Services.Interfaces;

namespace WPF.PacketCapture.Services
{

    public sealed class PacketCaptureService : IPacketCaptureService
    {

        #region Methods

        public IEnumerable<IPacketCaptureDeviceModel> GetAllDevices()
        {
            return CaptureDeviceList.Instance.Select(device => new PacketCaptureDeviceModel(device));
        }

        #endregion

        private sealed class PacketCaptureDeviceModel : IPacketCaptureDeviceModel
        {

            #region Events

            public event EventHandler<IPacketModel> PacketReceived;

            #endregion

            #region Fields

            private readonly ICaptureDevice _Device;

            private Task _Task;

            #endregion

            #region Constructors

            public PacketCaptureDeviceModel(ICaptureDevice device)
            {
                this._Device = device;
                this._Device.OnPacketArrival += this.OnPacketReceived;
            }

            #endregion

            #region Properties

            public string Description => this._Device.Description;

            public string Name => this._Device.Name;

            #endregion

            #region Methods

            public void StartCapture()
            {
                this._Device.StopCapture();

                // read timeout
                var timeout = 1000;
                this._Device.Open(DeviceMode.Promiscuous, timeout);
                this._Device.Filter = "ip and tcp and port http";

                this._Task?.Dispose();
                this._Task = Task.Run(() =>
                {
                    try
                    {
                        this._Device.StartCapture();
                    }
                    catch (Exception e)
                    {
                        Console.WriteLine(e);
                    }
                });
            }

            public void StopCapture()
            {
                try
                {
                    // 停止時に例外を投げるが原因が不明
                    this._Device.StopCapture();
                }
                catch (Exception e)
                {
                    Console.WriteLine(e);
                }

                this._Task?.Dispose();
            }

            #region Helper Methods

            private void OnPacketReceived(object sender, CaptureEventArgs captureEventArgs)
            {
                this.PacketReceived?.Invoke(this, new PacketModel(captureEventArgs.Packet));
            }

            #endregion

            #endregion
        }

        private sealed class PacketModel : IPacketModel
        {

            #region Fields

            private readonly Packet _Packet;

            private readonly RawCapture _RawCapture;

            #endregion

            #region Constructors

            public PacketModel(RawCapture rawPacket)
            {
                this._RawCapture = rawPacket;
                this._Packet = Packet.ParsePacket(rawPacket.LinkLayerType, rawPacket.Data);
            }

            #endregion

            #region Properties

            public string Destination => (this._Packet.PayloadPacket as IPv4Packet)?.DestinationAddress?.ToString();

            public int Length => this._Packet.Bytes.Length;

            public DateTime Timestamp => this._RawCapture.Timeval.Date;

            public string Source => (this._Packet.PayloadPacket as IPv4Packet)?.SourceAddress?.ToString();

            #endregion

        }

    }

}