using Xamarin.Camera.ViewModels.Interfaces;

namespace Xamarin.Camera.DesignTimes
{

    public sealed class ScreenPageViewModel : IScreenPageViewModel
    {

        #region Constructors

        public ScreenPageViewModel()
        {
            this.Title = "Screen";
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
