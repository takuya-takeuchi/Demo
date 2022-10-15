using Prism.Commands;

namespace Demo.ViewModels.Modules.Interfaces
{

    internal interface IModuleViewModel
    {

        bool Checked
        {
            get;
            set;
        }

        DelegateCommand ClickCommand
        {
            get;
        }

    }

}