using System;
using GalaSoft.MvvmLight;
using GalaSoft.MvvmLight.Command;
using WPFPython1.ViewModels.Interfaces;

namespace WPFPython1.ViewModels
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
                    this._MessageDialog.ShowMessage(this._PythonWrapper.ShowMessage());
                });
            }
        }

        #endregion

    }
}
