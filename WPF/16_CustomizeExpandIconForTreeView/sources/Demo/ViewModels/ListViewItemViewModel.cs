using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Demo.ViewModels
{

    internal sealed class ListViewItemViewModel
    {

        #region Constructors

        public ListViewItemViewModel(string text)
        {
            this.Text = text;
            this.Children = new ObservableCollection<ListViewItemViewModel>();
        }

        #endregion

        #region Properties

        public ObservableCollection<ListViewItemViewModel> Children
        {
            get;
        }

        public string Text
        {
            get;
        }

        #endregion

    }

}
