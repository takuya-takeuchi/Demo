using Prism.Commands;

using Demo.ViewModels.Interfaces;

namespace Demo.DesignTimes
{

    public sealed class MainPageViewModel : IMainPageViewModel
    {

        public DelegateCommand ReloadCommand
        {
            get;
        }

        public DelegateCommand ResetCommand
        {
            get;
        }

        public DelegateCommand ShowLogCommand
        {
            get;
        }

        public string Key
        {
            get;
        }

        public string Title
        {
            get;
        }

    }

}
