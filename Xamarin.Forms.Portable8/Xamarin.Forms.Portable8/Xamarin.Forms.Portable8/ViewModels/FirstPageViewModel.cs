using Prism.Commands;
using Prism.Common;
using Prism.Mvvm;
using Xamarin.Forms.Portable8.Views;

namespace Xamarin.Forms.Portable8.ViewModels
{
    public sealed class FirstPageViewModel : BindableBase, IPageAware
    {

        public Page Page
        {
            get;
            set;
        }

        private DelegateCommand _NextCommand;
        public DelegateCommand NextCommand
        {
            get
            {
                if (this._NextCommand == null)
                {
                    this._NextCommand = new DelegateCommand(() =>
                    {
                        this.Page.Navigation.PushAsync(new NextPage());
                    });
                }

                return this._NextCommand;
            }
        }
    }
}
