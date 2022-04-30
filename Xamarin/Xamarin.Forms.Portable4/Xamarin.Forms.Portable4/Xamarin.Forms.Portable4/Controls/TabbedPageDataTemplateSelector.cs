using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Xamarin.Forms.Portable4.ViewModels;

namespace Xamarin.Forms.Portable4.Controls
{
    public sealed class TabbedPageDataTemplateSelector : DataTemplateSelector
    {

        public TabbedPageDataTemplateSelector()
        {
            
        }

        public DataTemplate TabbedPage1
        {
            get;
            set;
        }

        public DataTemplate TabbedPage2
        {
            get;
            set;
        }

        protected override DataTemplate OnSelectTemplate(object item, BindableObject container)
        {
            var viewModel = item as TabbedPageViewModel;
            if (viewModel == null)
                return null;

            return viewModel is TabbedPage1ViewModel ? this.TabbedPage1 : viewModel is TabbedPage2ViewModel ? this.TabbedPage2 : null;
        }
    }
}
