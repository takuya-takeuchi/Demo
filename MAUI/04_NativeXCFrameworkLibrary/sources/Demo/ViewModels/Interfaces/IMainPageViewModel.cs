using Prism.Commands;

namespace Demo.ViewModels.Interfaces
{

    internal interface IMainPageViewModel
    {

        DelegateCommand InvokeNativeCommand
        {
            get;
        }

    }

}
