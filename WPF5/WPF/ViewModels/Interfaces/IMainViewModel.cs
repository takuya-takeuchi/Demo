using System.Collections.Generic;
using System.Windows.Data;
using GalaSoft.MvvmLight.Command;
using WPF.Models;

namespace WPF.ViewModels.Interfaces
{
    public interface IMainViewModel
    {

        int Column
        {
            get;
        }

        IEnumerable<GridSizeModel> GridSizes
        {
            get;
        }

        CollectionViewSource Items
        {
            get;
        }

        int Row
        {
            get;
        }

        RelayCommand<GridSizeModel> SelectGridSizeCommand
        {
            get;
        }

    }
}
