using System.Windows.Media;
using GalaSoft.MvvmLight.Command;

namespace Grpc1.ViewModels.Interfaces
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