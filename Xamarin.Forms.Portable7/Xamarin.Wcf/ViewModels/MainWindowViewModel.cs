using System;
using System.Collections.ObjectModel;
using System.Windows.Input;
using Microsoft.Practices.Prism.Mvvm;
using Prism.Commands;
using Xamarin.Wcf.Services;

namespace Xamarin.Wcf.ViewModels
{
    public sealed class MainWindowViewModel : BindableBase
    {
        private readonly IMessageService _MessageService;
        public MainWindowViewModel()
        {
            this.Messages = new ObservableCollection<MessageViewModel>();
        }

        public MainWindowViewModel(IMessageService messageService)
        {
            this._MessageService = messageService;
            this._MessageService.ReceivedImage += MessageServiceOnReceivedMessage;

            this.Messages = new ObservableCollection<MessageViewModel>();
        }

        private void MessageServiceOnReceivedMessage(object sender, MessageDto message)
        {
            this._Messages.Insert(0, new MessageViewModel { Message = message, DateTime = DateTime.Now.ToString("yyyy/MM/dd hh:mm:ss.fff") });
        }

        private ObservableCollection<MessageViewModel> _Messages;

        public ObservableCollection<MessageViewModel> Messages
        {
            get
            {
                return this._Messages;
            }
            private set
            {
                this.SetProperty(ref this._Messages, value);
            }
        }

        private DelegateCommand _ClearCommand;
        public ICommand ClearCommand
        {
            get
            {
                if (this._ClearCommand == null)
                {
                    this._ClearCommand = new DelegateCommand(
                        () =>
                        {
                            this._Messages.Clear();
                        },
                        () => true);
                }

                return this._ClearCommand;
            }
        }

    }

    public sealed class MessageViewModel : BindableBase
    {

        private MessageDto _Message;

        public MessageDto Message
        {
            get
            {
                return this._Message;
            }
            set
            {
                this.SetProperty(ref this._Message, value);
            }
        }

        private string _DateTime;

        public string DateTime
        {
            get
            {
                return this._DateTime;
            }
            set
            {
                this.SetProperty(ref this._DateTime, value);
            }
        }
    }
}
