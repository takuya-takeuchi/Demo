using Prism.Commands;

namespace Demo.ViewModels.Interfaces
{

    public interface IMainPageViewModel
    {

        string Address
        {
            get;
            set;
        }

        DelegateCommand<string> ConnectCommand
        {
            get;
        }

        DelegateCommand DisconnectCommand
        {
            get;
        }

        bool IsConnected
        {
            get;
        }

        string Title
        {
            get;
        }

    }

}