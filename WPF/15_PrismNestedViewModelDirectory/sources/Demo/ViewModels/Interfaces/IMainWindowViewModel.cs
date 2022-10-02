using Prism.Commands;

namespace Demo.ViewModels.Interfaces
{

    internal interface IMainWindowViewModel
    {

        DelegateCommand<string> ShowViewCommand { get; }

    }

}