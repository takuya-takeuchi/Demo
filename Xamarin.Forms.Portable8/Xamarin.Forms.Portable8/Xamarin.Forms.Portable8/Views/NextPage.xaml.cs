using Prism.Common;

namespace Xamarin.Forms.Portable8.Views
{
    public partial class NextPage : ContentPage
    {
        public NextPage()
        {
            // BindingContext is updated before InitializeComponent
            this.BindingContextChanged += (sender, args) =>
            {
                var pageAware = this.BindingContext as IPageAware;
                if (pageAware != null)
                {
                    pageAware.Page = this;
                }
            };

            InitializeComponent();
        }
    }
}
