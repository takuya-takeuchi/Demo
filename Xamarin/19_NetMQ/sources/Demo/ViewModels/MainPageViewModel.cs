using System;

using Prism.Navigation;
using Prism.Commands;

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

            this._ZeroMqSubscribeService = zeroMqSubscribeService;
        }

        #endregion

        #region Properties

        private string _Address;

        public string Address
        {
            get => this._Address;
            set => SetProperty(ref this._Address, value);
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
                        this.IsConnected = true;
                    }
                    catch (Exception e)
                    {
                    }
                }, s => !string.IsNullOrWhiteSpace(s)).ObservesProperty(() => this.Address));
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

        private string _Title;

        public string Title
        {
            get => this._Title;
            private set => SetProperty(ref this._Title, value);
        }

        #endregion

        #region Methods

        #region Event Handlers

        private void Callback(string obj)
        {
            throw new NotImplementedException();
        }

        #endregion

        #region Helpers
        #endregion

        #endregion

    }

}
