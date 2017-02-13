using System;
using System.Linq;
using GalaSoft.MvvmLight;
using GalaSoft.MvvmLight.Command;
using WPFPython2.ViewModels.Interfaces;

namespace WPFPython2.ViewModels
{

    public sealed class MainViewModel : ViewModelBase, IMainViewModel
    {

        #region フィールド

        private readonly IPythonWrapper _PythonWrapper;

        private readonly IMessageDialog _MessageDialog;

        #endregion

        #region コンストラクタ

        public MainViewModel(IPythonWrapper pythonWrapper, IMessageDialog messageDialog)
        {
            if (pythonWrapper == null)
                throw new ArgumentNullException(nameof(pythonWrapper));
            if (messageDialog == null)
                throw new ArgumentNullException(nameof(messageDialog));

            this._PythonWrapper = pythonWrapper;
            this._MessageDialog = messageDialog;

            this.Start = 1;
            this.End = 10;
        }

        #endregion

        #region プロパティ

        private RelayCommand _MessageCommand;

        public RelayCommand MessageCommand
        {
            get
            {
                return this._MessageCommand ?? new RelayCommand(() =>
                {
                    var array = Enumerable.Range(this._Start, this._End - this._Start + 1).ToArray();
                    var sum = array.Sum();
                    var list = this._PythonWrapper.Twice(array);

                    var message = $"Initial total:{sum}, final total:{list.Sum()}";
                    this._MessageDialog.ShowMessage(message);
                });
            }
        }

        private int _Start;

        public int Start
        {
            get
            {
                return this._Start;
            }
            set
            {
                this._Start = value;
                this.RaisePropertyChanged();
            }
        }

        private int _End;

        public int End
        {
            get
            {
                return this._End;
            }
            set
            {
                this._End = value;
                this.RaisePropertyChanged();
            }
        }

        #endregion

    }
}
