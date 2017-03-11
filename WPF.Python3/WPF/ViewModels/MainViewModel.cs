using System;
using System.Collections.ObjectModel;
using System.Linq;
using GalaSoft.MvvmLight;
using GalaSoft.MvvmLight.Command;
using WPFPython.ViewModels.Interfaces;

namespace WPFPython.ViewModels
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

            var keys = new string[]
            {
                "name",
                "birth",
                "dead",
                "nation",
                "publish",
            };

            this._Keys = new ObservableCollection<string>(keys);
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
                    var dict = this._PythonWrapper.load("sample.json");

                    //var message = $"Initial total:{sum}, final total:{list.Sum()}";
                    //this._MessageDialog.ShowMessage(message);
                });
            }
        }

        private string _SelectedKey;

        public string SelectedKey
        {
            get
            {
                return this._SelectedKey;
            }
            set
            {
                this._SelectedKey = value;
                this.RaisePropertyChanged();
            }
        }

        private ObservableCollection<string> _Keys;

        public ObservableCollection<string> Keys
        {
            get
            {
                return this._Keys;
            }
        }

        #endregion

    }
}
