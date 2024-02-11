using Prism.Commands;

namespace Demo.ViewModels.Interfaces
{

    internal interface IMainWindowViewModel
    {

        DelegateCommand LoadedCommand { get; }

        string RegionName
        {
            get;
        }

    }

}