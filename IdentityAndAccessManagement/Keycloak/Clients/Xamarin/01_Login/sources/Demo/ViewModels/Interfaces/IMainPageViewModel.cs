using Prism.Commands;

namespace Demo.ViewModels.Interfaces
{

    public interface IMainPageViewModel : IViewModel
    {

        DelegateCommand RefreshCommand
        {
            get;
        }

        DelegateCommand ShowLogCommand
        {
            get;
        }

        string AccessTokenExpire
        {
            get;
        }

    }

}