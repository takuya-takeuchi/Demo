using GalaSoft.MvvmLight;
using WPF1.ViewModels.Interfaces;

namespace WPF1.ViewModels
{

    public sealed class MainViewModel : ViewModelBase, IMainViewModel
    {

        #region フィールド
        #endregion

        #region コンストラクタ

        public MainViewModel()
        {
            this.HorizontalCount = 3;
            this.VerticalCount = 3;
            this.BorderSize = 30;

            this.UpdateCheckState();
        }

        #endregion

        #region プロパティ

        private int _BorderSize;

        public int BorderSize
        {
            get
            {
                return this._BorderSize;
            }
            set
            {
                this._BorderSize = value;
                this.RaisePropertyChanged();
            }
        }

        private bool[,] _CheckState;

        public bool[,] CellStates
        {
            get
            {
                return this._CheckState;
            }
            private set
            {
                this._CheckState = value;
                this.RaisePropertyChanged();
            }
        }

        private int _HorizontalCount;

        public int HorizontalCount
        {
            get
            {
                return this._HorizontalCount;
            }
            set
            {
                this._HorizontalCount = value;
                this.RaisePropertyChanged();
                this.UpdateCheckState();
            }
        }

        private int _VerticalCount;

        public int VerticalCount
        {
            get
            {
                return this._VerticalCount;
            }
            set
            {
                this._VerticalCount = value;
                this.RaisePropertyChanged();
                this.UpdateCheckState();
            }
        }

        #endregion

        #region メソッド

        #region オーバーライド
        #endregion

        #region イベントハンドラ
        #endregion

        #region ヘルパーメソッド

        private void UpdateCheckState()
        {
            this.CellStates = new bool[this.HorizontalCount, this.VerticalCount];
        }

        #endregion

        #endregion
    }

}
