using Microsoft.Practices.Prism.Mvvm;

namespace Xamarin.Forms.Portable1.ViewModels
{
    public sealed class MainPageViewModel : BindableBase
    {

        public MainPageViewModel()
        {
            this.MainText = "Hello world from MVVM!!";
        }

        private string _MainText;

        public string MainText
        {
            get
            {
                return this._MainText;
            }
            set
            {
                this._MainText = value;
                this.SetProperty(ref this._MainText, value);
            }
        }
    }
}
