using System.Windows.Input;
using Xamarin.OpenCV.ViewModels.Interfaces;

namespace Xamarin.OpenCV.DesignTimes
{

    public class InfoPageViewModel : IInfoPageViewModel
    {

        #region Constructors

        public InfoPageViewModel()
        {
            this.Title = "Main Page";
            this.BuildInformation = "BuildInformation";
            this.Version ="4.3.0";
        }

        #endregion

        #region Properties

        public ICommand ShowBuildInformation
        {
            get;
        }

        public string BuildInformation
        {
            get;
        }

        public string Title
        {
            get;
        }

        public string Version
        {
            get;
        }

        #endregion

    }

}
