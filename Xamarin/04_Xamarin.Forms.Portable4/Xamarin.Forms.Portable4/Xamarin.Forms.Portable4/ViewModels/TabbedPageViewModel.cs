using Microsoft.Practices.Prism.Mvvm;

namespace Xamarin.Forms.Portable4.ViewModels
{
    public abstract class TabbedPageViewModel : BindableBase
    {

        private string _Title;

        public string Title
        {
            get
            {
                return this._Title;
            }
            protected set
            {
                this.SetProperty(ref this._Title, value);
            }
        }

    }

    public sealed class TabbedPage1ViewModel : TabbedPageViewModel
    {

        public TabbedPage1ViewModel()
        {
            this.Title = "TabbedPage1";
        }

    }
    public sealed class TabbedPage2ViewModel : TabbedPageViewModel
    {
        public TabbedPage2ViewModel()
        {
            this.Title = "TabbedPage2";
        }

    }

}
