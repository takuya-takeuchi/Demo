using Microsoft.Practices.Prism.Mvvm;

namespace Shared.ViewModels
{

    public sealed class MainWindowViewModel : BindableBase
    {

        private string _Text1;

        public string Text1
        {
            get
            {
                return this._Text1;
            }
            set
            {
                SetProperty(ref this._Text1, value);
            }
        }

        private string _Text2;

        public string Text2
        {
            get
            {
                return this._Text2;
            }
            set
            {
                SetProperty(ref this._Text2, value);
            }
        }

        public void OnScrollChanged()
        {
            var random = new System.Random();
            this.Text1 = random.Next(0, 10000).ToString();
            this.Text2 = random.Next(0, 10000).ToString();
        }

    }

}
