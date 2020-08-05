namespace Xamarin.Native.Nuget.ViewModels.Interfaces
{

    public interface IMainPageViewModel
    {

        string Title
        {
            get;
        }

        int Left
        {
            get;
            set;
        }

        int Right
        {
            get;
            set;
        }

        int AddResult
        {
            get;
        }

        int MulResult
        {
            get;
        }

        ICommand CalcCommand
        {
            get;
        }

    }

}
