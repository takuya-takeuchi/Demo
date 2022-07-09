using Prism.Commands;

using Demo.ViewModels.Interfaces;

namespace Demo.DesignTimeData
{

    internal class MainPageViewModel : IMainPageViewModel
    {

        #region Constructors

        public MainPageViewModel()
        {
            this.LeftOperand = 10;
            this.RightOperand = 2;
            this.Operator = "?";
        }

        #endregion

        #region IMainPageViewModel Members

        public DelegateCommand AddCommand
        {
            get;
        }

        public DelegateCommand SubtractCommand
        {
            get;
        }

        public DelegateCommand MultiplyCommand
        {
            get;
        }

        public DelegateCommand DivideCommand
        {
            get;
        }

        public int LeftOperand
        {
            get;
            set;
        }

        public int RightOperand
        {
            get;
            set;
        }

        public int Result
        {
            get;
        }

        public string Operator
        {
            get;
        }

        #endregion

    }

}