using System;
using System.Collections.ObjectModel;

using Prism.Navigation;
using Prism.Commands;
using Xamarin.Essentials;

using Demo.Services.Interfaces;
using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    public sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Fields

        private readonly IZeroMQSubscribeService _ZeroMqSubscribeService;

        #endregion

        #region Constructors

        public MainPageViewModel(INavigationService navigationService,
                                 IZeroMQSubscribeService zeroMqSubscribeService)
            : base(navigationService)
        {
            this.Title = "Main Page";
            this.Address = "tcp://192.168.11.21:12345";

            this._ZeroMqSubscribeService = zeroMqSubscribeService;
            this._ZeroMqSubscribeService.StateChanged += this.StateChanged;
        }

        #endregion

        #region Properties

        private string _Address;

        public string Address
        {
            get => this._Address;
            set => SetProperty(ref this._Address, value);
        }

        private DelegateCommand _ClearCommand;

        public DelegateCommand ClearCommand
        {
            get
            {
                return this._ClearCommand ?? (this._ClearCommand = new DelegateCommand(() =>
                {
                    this._Messages.Clear();
                }));
            }
        }

        private DelegateCommand<string> _ConnectCommand;

        public DelegateCommand<string> ConnectCommand
        {
            get
            {
                return this._ConnectCommand ?? (this._ConnectCommand = new DelegateCommand<string>(s =>
                {
                    try
                    {
                        this.IsConnected = false;
                        this._ZeroMqSubscribeService.Connect(s, this.Callback);
                    }
                    catch (Exception e)
                    {
                        this._ZeroMqSubscribeService.Disconnect();
                    }
                }, s => !string.IsNullOrWhiteSpace(s) && !this.IsConnected).ObservesProperty(() => this.Address)
                                                                           .ObservesProperty(() => this.IsConnected));
            }
        }

        private DelegateCommand _DisconnectCommand;

        public DelegateCommand DisconnectCommand
        {
            get
            {
                return this._DisconnectCommand ?? (this._DisconnectCommand = new DelegateCommand(() =>
                {
                    this._ZeroMqSubscribeService.Disconnect();
                }, () => this.IsConnected).ObservesProperty(() => this.IsConnected));
            }
        }

        private bool _IsConnected;

        public bool IsConnected
        {
            get => this._IsConnected;
            private set => SetProperty(ref this._IsConnected, value);
        }

        private readonly ObservableCollection<string> _Messages = new ObservableCollection<string>();

        public ObservableCollection<string> Messages
        {
            get => this._Messages;
        }

        private DelegateCommand _ShowLogCommand;

        public DelegateCommand ShowLogCommand
        {
            get
            {
                return this._ShowLogCommand ?? (this._ShowLogCommand = new DelegateCommand(() =>
                {
                    this.NavigationService.NavigateAsync("ShowLogs");
                }));
            }
        }

        #endregion

        #region Methods

        #region Event Handlers

        private void Callback(string obj)
        {
            MainThread.BeginInvokeOnMainThread(() =>
            {
                this._Messages.Add(obj);
            });
        }

        private void StateChanged(object sender, bool state)
        {
            this.IsConnected = state;
        }

        #endregion

        #endregion

    }

}
