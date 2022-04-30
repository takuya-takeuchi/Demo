using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using PcapDotNet.Core;
using PcapDotNet.Packets;
using WPF.PcapNet.Models.Interfaces;
using WPF.PcapNet.Services.Interfaces;

namespace WPF.PcapNet.Services
{

    public sealed class PacketCaptureService : IPacketCaptureService
    {

        #region Methods

        public IEnumerable<IPacketCaptureDeviceModel> GetAllDevices()
        {
            return LivePacketDevice.AllLocalMachine.Select(device => new PacketCaptureDeviceModel(device));
        }

        #endregion

        private sealed class PacketCaptureDeviceModel : IPacketCaptureDeviceModel
        {

            #region Events

            public event EventHandler<IPacketModel> PacketReceived;

            #endregion

            #region Fields

            private readonly LivePacketDevice _Device;

            private BerkeleyPacketFilter _Filter;

            private PacketCommunicator _PacketCommunicator;

            private Task _Task;

            #endregion

            #region Constructors

            public PacketCaptureDeviceModel(LivePacketDevice device)
            {
                this._Device = device;
            }

            #endregion

            #region Properties

            public string Description => this._Device.Description;

            public string Name => this._Device.Name;

            #endregion

            #region Methods

            public void StartCapture()
            {
                // portion of the packet to capture
                // 65536 guarantees that the whole packet will be captured on all the link layers
                var snapshotLength = 65536;

                // promiscuous mode
                var packetDeviceOpenAttributes = PacketDeviceOpenAttributes.Promiscuous;

                // read timeout
                var timeout = 1000;

                this._PacketCommunicator?.Break();
                this._PacketCommunicator?.Dispose();
                this._PacketCommunicator = this._Device.Open(snapshotLength, packetDeviceOpenAttributes, timeout);

                this._Filter?.Dispose();

                this._Filter = this._PacketCommunicator.CreateFilter("ip and tcp and port http");
                this._PacketCommunicator.SetFilter(this._Filter);

                this._Task?.Dispose();
                this._Task = Task.Run(() =>
                {
                    try
                    {
                        // 停止時に例外を投げるが、原因不明
                        this._PacketCommunicator.ReceivePackets(0, this.OnPacketReceived);
                    }
                    catch (Exception e)
                    {
                        Console.WriteLine(e);
                    }
                });
            }

            public void StopCapture()
            {
                this._PacketCommunicator?.Break();
                this._PacketCommunicator?.Dispose();
                this._Filter?.Dispose();

                this._PacketCommunicator = null;
                this._Filter = null;
            }

            #region Helper Methods

            private void OnPacketReceived(Packet packet)
            {
                this.PacketReceived?.Invoke(this, new PacketModel(packet));
            }

            #endregion

            #endregion
        }

        private sealed class PacketModel : IPacketModel
        {

            #region Fields

            private readonly Packet _Packet;

            #endregion

            #region Constructors

            public PacketModel(Packet packet)
            {
                this._Packet = packet;
            }

            #endregion

            #region Properties

            public string Destination => this._Packet.IpV4.Destination.ToString();

            public int Length => this._Packet.Length;

            public DateTime Timestamp => this._Packet.Timestamp;

            public string Source => this._Packet.IpV4.Source.ToString();

            #endregion

        }

    }

}