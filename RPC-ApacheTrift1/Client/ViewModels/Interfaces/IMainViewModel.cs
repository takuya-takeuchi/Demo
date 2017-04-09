using System.Windows.Media;
using GalaSoft.MvvmLight.Command;

namespace ImageProcClient.ViewModels.Interfaces
{

    public interface IMainViewModel
    {

        #region Properties

        RelayCommand OpenFileCommand
        {
            get;
        }

        ImageSource ResultImage
        {
            get;
        }

        RelayCommand ServerRequestCommand
        {
            get;
        }

        ImageSource SourceImage
        {
            get;
        }

        #endregion

    }

}