using System.Collections.ObjectModel;
using System.Windows.Input;

using Demo.Models;
using Demo.ViewModels.Interfaces;

namespace Demo.DesignTimes
{

    public sealed class MediaPageViewModel : IMediaPageViewModel
    {

        public string Title
        {
            get;
        }

        public ICommand ItemTappedCommand
        {
            get;
        }

        public ObservableCollection<MediaAsset> MediaAssets
        {
            get;
            set;
        }

        public MediaAsset MediaSelected
        {
            get;
            set;
        }

        public string SearchText
        {
            get;
            set;
        }

    }

}