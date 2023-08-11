namespace Demo.ViewModels.Interfaces
{

    internal interface IMainPageViewModel
    {

        string CardNumber
        {
            get;
            set;
        }

        bool IsValid
        {
            get;
        }

    }

}
