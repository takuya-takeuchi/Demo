﻿using System.Collections.ObjectModel;

using Prism.Commands;

namespace Demo.ViewModels.Interfaces
{

    public interface IMainPageViewModel : IViewModel
    {

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

        DelegateCommand ShowLogCommand
        {
            get;
        }

    }

}