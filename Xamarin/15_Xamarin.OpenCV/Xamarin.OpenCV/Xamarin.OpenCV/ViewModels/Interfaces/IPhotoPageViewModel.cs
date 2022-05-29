using System.Windows.Input;
using Xamarin.Forms;

namespace Xamarin.OpenCV.ViewModels.Interfaces
{

    public interface IPhotoPageViewModel
    {

        ICommand ShowPhoto
        {
            get;
        }

        string Title
        {
            get;
        }

        ImageSource Photo
        {
            get;
        }

    }

}