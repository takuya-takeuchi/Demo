namespace Demo.ViewModels.Interfaces
{

    internal interface IMainPageViewModel
    {

        DelegateCommand ClickCommand
        {
            get;
        }

        string Text
        {
            get;
        }

    }

}
