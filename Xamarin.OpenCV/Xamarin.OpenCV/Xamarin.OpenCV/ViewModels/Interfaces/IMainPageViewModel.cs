using System.Windows.Input;
using Prism.Commands;

namespace Xamarin.OpenCV.ViewModels.Interfaces
{

    public interface IMainPageViewModel
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