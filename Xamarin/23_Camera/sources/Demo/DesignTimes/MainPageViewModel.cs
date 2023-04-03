using Xamarin.Forms;

using Prism.Commands;

using Demo.ViewModels.Interfaces;

namespace Demo.DesignTimes
{

    public sealed class MainPageViewModel : IMainPageViewModel
    {

        public ImageSource PhotoImage
        {
            get;
        }

        public DelegateCommand ShowLogCommand
        {
            get;
        }

        public DelegateCommand TakePhotoCommand
        {
            get;
        }

        public string Title
        {
            get;
        }

    }

}
