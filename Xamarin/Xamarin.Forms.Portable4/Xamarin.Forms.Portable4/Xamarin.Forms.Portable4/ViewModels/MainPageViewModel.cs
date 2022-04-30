using System.Collections.ObjectModel;
using Microsoft.Practices.Prism.Mvvm;

namespace Xamarin.Forms.Portable4.ViewModels
{
    public sealed class MainPageViewModel : BindableBase
    {
        public MainPageViewModel()
        {
            var tabbedPageViewModels = new TabbedPageViewModel[]
            {
                new TabbedPage1ViewModel(),
                new TabbedPage2ViewModel(),
            };

            this.TabbedPages = new ObservableCollection<TabbedPageViewModel>(tabbedPageViewModels);
        }

        private ObservableCollection<TabbedPageViewModel> _TabbedPages;

        public ObservableCollection<TabbedPageViewModel> TabbedPages
        {
            get
            {
                return this._TabbedPages;
            }
            protected set
            {
                this.SetProperty(ref this._TabbedPages, value);
            }
        }

    }

}