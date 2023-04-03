using Xamarin.Forms;

using Prism.Commands;

namespace Demo.ViewModels.Interfaces
{

    public interface IMainPageViewModel : IViewModel
    {

        ImageSource PhotoImage
        {
            get;
        }

        DelegateCommand ShowLogCommand
        {
            get;
        }

        DelegateCommand TakePhotoCommand
        {
            get;
        }

    }

}