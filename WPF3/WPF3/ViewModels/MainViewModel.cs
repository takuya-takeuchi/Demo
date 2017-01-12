using GalaSoft.MvvmLight;
using WPF3.ViewModels.Interfaces;

namespace WPF3.ViewModels
{

    public sealed class MainViewModel : ViewModelBase, IMainViewModel
    {

        #region コンストラクタ

        public MainViewModel()
        {
        }

        #endregion

        #region プロパティ

        private bool _State1;

        public bool State1
        {
            get
            {
                return this._State1;
            }
            set
            {
                this._State1 = value;
                this.RaisePropertyChanged();
            }
        }

        private bool _State2;

        public bool State2
        {
            get
            {
                return this._State2;
            }
            set
            {
                this._State2 = value;
                this.RaisePropertyChanged();
            }
        }

        #endregion

    }

}
