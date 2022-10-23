using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    internal sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Fields

        private int _Count = 0;

        #endregion

        #region Constructors

        public MainPageViewModel(INavigationService navigationService)
            : base(navigationService)
        {
            this.Text = "Click me";
        }

        #endregion

        #region IMainPageViewModel Members

        private DelegateCommand _ClickCommand;

        public DelegateCommand ClickCommand
        {
            get
            {
                return this._ClickCommand ??= new DelegateCommand(() =>
                {
                    this._Count++;
                    this.Text = this._Count == 1 ? $"Clicked {this._Count} time" : $"Clicked {this._Count} times";
                });
            }
        }

        private string _Text;

        public string Text
        {
            get => this._Text;
            private set => this.SetProperty(ref this._Text, value);
        }

        #endregion

    }

}