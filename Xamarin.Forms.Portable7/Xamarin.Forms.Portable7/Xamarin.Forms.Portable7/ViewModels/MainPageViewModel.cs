using System;
using System.ServiceModel;
using System.Windows.Input;
using Microsoft.Practices.Prism.Commands;
using Microsoft.Practices.Prism.Mvvm;
using Xamarin.Forms.Portable7.Services;

namespace Xamarin.Forms.Portable7.ViewModels
{

    public sealed class MainPageViewModel : BindableBase
    {

        #region Fields

        private MessageServiceClient _MessageServiceClient;

        #endregion

        #region Properties

        private string _IpAddress;

        public string IpAddress
        {
            get
            {
                return this._IpAddress;
            }
            set
            {
                this.SetProperty(ref this._IpAddress, value);

                this._SendCommand?.RaiseCanExecuteChanged();
            }
        }


        private string _Message;

        public string Message
        {
            get
            {
                return this._Message;
            }
            set
            {
                var changed = this._Message != value;
                this.SetProperty(ref this._Message, value);

                if (changed && this._MessageServiceClient != null)
                {
                    this._MessageServiceClient.CloseAsync();
                    this._MessageServiceClient = null;
                }
            }
        }

        private DelegateCommand _SendCommand;
        public ICommand SendCommand
        {
            get
            {
                if (this._SendCommand == null)
                {
                    this._SendCommand = new DelegateCommand(
                        () =>
                        {
                            this.UpdateServiceClient();

                            var message = new messageDto();
                            message.message = this._Message;
                            message.sender = ServiceManager.Sender;
                            this._MessageServiceClient.SendMessageAsync(message);
                        },
                        () =>
                        {
                            return !string.IsNullOrWhiteSpace(this.IpAddress);
                        });
                }

                return this._SendCommand;
            }
        }

        #endregion

        #region Methods

        private void UpdateServiceClient()
        {
            if (this._MessageServiceClient != null)
            {
                return;
            }

            var binding = new BasicHttpBinding()
            {
                Name = "basicHttpBinding",
                MaxReceivedMessageSize = 67108864,
            };

            var timeout = new TimeSpan(0, 1, 0);
            binding.SendTimeout = timeout;
            binding.OpenTimeout = timeout;
            binding.ReceiveTimeout = timeout;
            this._MessageServiceClient = new MessageServiceClient(
                binding,
                new EndpointAddress($"http://{this.IpAddress}:8080/Xamarin.Wcf/MessageService"));
        }

        #endregion

    }

}
