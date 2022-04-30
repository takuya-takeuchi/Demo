using Xamarin.OpenCV.ViewModels.Interfaces;

namespace Xamarin.OpenCV.DesignTimes
{

    public class MainPageViewModel : IMainPageViewModel
    {

        #region Constructors

        public MainPageViewModel()
        {
            this.Title = "Info";
        }

        #endregion

        #region Properties

        public string Title
        {
            get;
        }

        #endregion

    }

}
