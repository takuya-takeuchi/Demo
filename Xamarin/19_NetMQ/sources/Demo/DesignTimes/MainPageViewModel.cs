using Prism.Commands;

using Demo.ViewModels.Interfaces;

namespace Demo.DesignTimes
{

    public sealed class MainPageViewModel : IMainPageViewModel
    {

        public string Address
        {
            get;
            set;
        }

        public DelegateCommand<string> ConnectCommand
        {
            get;
        }

        public DelegateCommand DisconnectCommand
        {
            get;
        }

        public bool IsConnected
        {
            get;
        }

        public string Title
        {
            get;
        }

    }

}