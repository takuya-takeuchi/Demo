using System.Windows.Input;
using Prism.Commands;
using Xamarin.OpenCV.ViewModels.Interfaces;

namespace Xamarin.OpenCV.DesignTimes
{

    public class MainPageViewModel : IMainPageViewModel
    {

        #region Constructors

        public MainPageViewModel()
        {
            this.Title = "Main Page";
            this.BuildInformation = "BuildInformation";
            this.Version ="4.3.0";
        }

        #endregion

        #region Properties

        private DelegateCommand _ShowBuildInformation;

        public ICommand ShowBuildInformation
        {
            get
            {
                return this._ShowBuildInformation ?? (this._ShowBuildInformation = new DelegateCommand(() =>
                {
                }));
            }
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
