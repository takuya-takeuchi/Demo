using Prism.Commands;

namespace Demo.ViewModels.Interfaces
{

    public interface IMainPageViewModel : IViewModel
    {

        bool CameraOpened
        {
            get;
        }

        DelegateCommand ShowLogCommand
        {
            get;
        }

        DelegateCommand CameraPreviewCommand
        {
            get;
        }

    }

}