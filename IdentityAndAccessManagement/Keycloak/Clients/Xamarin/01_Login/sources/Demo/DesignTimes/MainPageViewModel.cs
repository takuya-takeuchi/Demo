using Prism.Commands;

using Demo.ViewModels.Interfaces;

namespace Demo.DesignTimes
{

    public sealed class MainPageViewModel : IMainPageViewModel
    {

        public string AccessTokenExpire
        {
            get;
        }

        public DelegateCommand RefreshCommand
        {
            get;
        }

        public DelegateCommand ShowLogCommand
        {
            get;
        }

        public string Title
        {
            get;
        }

    }

}
