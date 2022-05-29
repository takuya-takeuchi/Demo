using Xamarin.Camera.ViewModels.Interfaces;

namespace Xamarin.Camera.DesignTimes
{

    public sealed class MainPageViewModel : IMainPageViewModel
    {

        #region Constructors

        public MainPageViewModel()
        {
            this.Title = "Main Page";
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
