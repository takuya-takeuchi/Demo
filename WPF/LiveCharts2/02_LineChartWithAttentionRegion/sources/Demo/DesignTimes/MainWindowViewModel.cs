using Prism.Commands;

using Demo.ViewModels.Interfaces;

namespace Demo.DesignTimes
{

    internal class MainWindowViewModel : IMainWindowViewModel
    {

        public DelegateCommand LoadedCommand
        {
            get;
        }

        public string RegionName
        {
            get;
        }

    }

}
