using System.Windows.Input;
using Xamarin.Forms;
using Xamarin.OpenCV.ViewModels.Interfaces;

namespace Xamarin.OpenCV.DesignTimes
{

    public class PhotoPageViewModel : IPhotoPageViewModel
    {

        #region Constructors

        public PhotoPageViewModel()
        {
            this.Title = "Main Page";
        }

        #endregion

        #region Properties

        public ICommand ShowPhoto
        {
            get;
        }

        public string Title
        {
            get;
        }

        public ImageSource Photo
        {
            get;
        }

        #endregion

    }

}
