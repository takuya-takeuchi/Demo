using System.Windows.Input;

namespace Xamarin.OpenCV.ViewModels.Interfaces
{

    public interface IInfoPageViewModel
    {

        ICommand ShowBuildInformation
        {
            get;
        }

        string BuildInformation
        {
            get;
        }

        string Title
        {
            get;
        }

        string Version
        {
            get;
        }

    }

}