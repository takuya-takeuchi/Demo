using System.Collections.ObjectModel;

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

        ObservableCollection<string> Messages
        {
            get;
        }

        string Title
        {
            get;
        }

    }

}