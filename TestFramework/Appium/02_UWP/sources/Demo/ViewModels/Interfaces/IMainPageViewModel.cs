using Prism.Commands;

namespace Demo.ViewModels.Interfaces
{

    internal interface IMainPageViewModel
    {

        DelegateCommand AddCommand
        {
            get;
        }

        DelegateCommand SubtractCommand
        {
            get;
        }

        DelegateCommand MultiplyCommand
        {
            get;
        }

        DelegateCommand DivideCommand
        {
            get;
        }

        int LeftOperand
        {
            get;
            set;
        }

        int RightOperand
        {
            get;
            set;
        }

        int Result
        {
            get;
        }

        string Operator
        {
            get;
        }

    }

}