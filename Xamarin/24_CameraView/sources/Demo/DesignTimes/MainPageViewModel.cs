using Demo.Controls;
using Prism.Commands;

using Demo.ViewModels.Interfaces;

namespace Demo.DesignTimes
{

    public sealed class MainPageViewModel : IMainPageViewModel
    {

        public bool CameraOpened
        {
            get;
        }

        public Orientation CameraOrientation
        {
            get;
        }

        public DelegateCommand ShowLogCommand
        {
            get;
        }

        public DelegateCommand CameraPreviewCommand
        {
            get;
        }

        public string Title
        {
            get;
        }

    }

}
