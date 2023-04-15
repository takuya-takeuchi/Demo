using Prism.Commands;

using Demo.Controls;

namespace Demo.ViewModels.Interfaces
{

    public interface IMainPageViewModel : IViewModel
    {

        bool CameraOpened
        {
            get;
        }

        Orientation CameraOrientation
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