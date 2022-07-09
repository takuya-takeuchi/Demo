using System.Collections.ObjectModel;
using System.Windows.Input;

namespace TestApp.ViewModels.Interfaces
{

    public interface IMainWindowViewModel
    {

        string Title
        {
            get;
        }

        ICommand GeneratePrimeNumberCommand
        {
            get;
        }

        ICommand GenerateNonPrimeNumberCommand
        {
            get;
        }

        ObservableCollection<uint> Numbers
        {
            get;
        }

    }

}