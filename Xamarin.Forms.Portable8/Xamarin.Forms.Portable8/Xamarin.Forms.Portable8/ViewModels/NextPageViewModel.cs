using Prism.Commands;
using Prism.Common;
using Prism.Mvvm;

namespace Xamarin.Forms.Portable8.ViewModels
{
    public sealed class NextPageViewModel : BindableBase, IPageAware
    {
        public Page Page
        {
            get;
            set;
        }

        private DelegateCommand _BackCommand;
        public DelegateCommand BackCommand
        {
            get
            {
                if (this._BackCommand == null)
                {
                    this._BackCommand = new DelegateCommand(() =>
                    {
                        this.Page.Navigation.PopAsync();
                    });
                }

                return this._BackCommand;
            }
        }

    }
}