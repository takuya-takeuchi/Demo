using Prism.Commands;

using Demo.ViewModels.Modules.Interfaces;

namespace Demo.DesignTimes.Modules
{

    internal sealed class MainModuleViewModel : IModuleViewModel
    {

        public bool Checked
        {
            get;
            set;
        }

        public DelegateCommand ClickCommand
        {
            get;
        }

    }

}
