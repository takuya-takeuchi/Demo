using Prism.Windows.Mvvm;

namespace Demo.ViewModels
{

    internal sealed class MainPageViewModel : ViewModelBase
    {

        #region Constructors

        public MainPageViewModel()
        {
            this.Text = "Demo";
        }

        #endregion

        #region Properties

        private string _Text;

        public string Text
        {
            get => this._Text;
            set => this.SetProperty(ref this._Text, value);
        }

        #endregion

    }

}
