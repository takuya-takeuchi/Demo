namespace WPF1.ViewModels.Interfaces
{
    public interface IMainViewModel
    {

        int BorderSize
        {
            get;
            set;
        }

        bool[,] CellStates
        {
            get;
        }

        int HorizontalCount
        {
            get;
            set;
        }

        int VerticalCount
        {
            get;
            set;
        }

    }
}
