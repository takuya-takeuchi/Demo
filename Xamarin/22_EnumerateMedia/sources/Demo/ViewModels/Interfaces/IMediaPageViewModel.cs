using System.Collections.ObjectModel;
using System.Windows.Input;

using Demo.Models;

namespace Demo.ViewModels.Interfaces
{

    public interface IMediaPageViewModel : IViewModel
    {

        ICommand ItemTappedCommand { get; }

        ObservableCollection<MediaAsset> MediaAssets { get; set; }
        
        MediaAsset MediaSelected { get; set; }

        string SearchText { get; set; }

    }

}