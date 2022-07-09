using Prism.Commands;
using Prism.Windows.Mvvm;

using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    internal class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Constructors

        public MainPageViewModel()
        {
            this.Operator = "?";
        }

        #endregion

        #region IMainPageViewModel Members

        private DelegateCommand _AddCommand;

        public DelegateCommand AddCommand
        {
            get
            {
                return this._AddCommand ?? (this._AddCommand = new DelegateCommand(() =>
                {
                    this.Result = this._LeftOperand + this._RightOperand;
                    this.Operator = "+";
                }));
            }
        }

        private DelegateCommand _SubtractCommand;

        public DelegateCommand SubtractCommand
        {
            get
            {
                return this._SubtractCommand ?? (this._SubtractCommand = new DelegateCommand(() =>
                {
                    this.Result = this._LeftOperand - this._RightOperand;
                    this.Operator = "-";
                }));
            }
        }

        private DelegateCommand _MultiplyCommand;

        public DelegateCommand MultiplyCommand
        {
            get
            {
                return this._MultiplyCommand ?? (this._MultiplyCommand = new DelegateCommand(() =>
                {
                    this.Result = this._LeftOperand * this._RightOperand;
                    this.Operator = "*";
                }));
            }
        }

        private DelegateCommand _DivideCommand;

        public DelegateCommand DivideCommand
        {
            get
            {
                return this._DivideCommand ?? (this._DivideCommand = new DelegateCommand(() =>
                {
                    this.Result = this._LeftOperand / this._RightOperand;
                    this.Operator = "/";
                }, () => this.RightOperand != 0).ObservesProperty(() => this.RightOperand));

            }
        }

        private int _LeftOperand;

        public int LeftOperand
        {
            get => this._LeftOperand;
            set => this.SetProperty(ref this._LeftOperand, value);
        }

        private int _RightOperand;

        public int RightOperand
        {
            get => this._RightOperand;
            set => this.SetProperty(ref this._RightOperand, value);
        }

        private int _Result;

        public int Result
        {
            get => this._Result;
            private set => this.SetProperty(ref this._Result, value);
        }

        private string _Operator;

        public string Operator
        {
            get => this._Operator;
            private set => this.SetProperty(ref this._Operator, value);
        }

        #endregion

    }

}