using System.Windows.Input;
using Microsoft.Practices.Prism.Mvvm;
using Xamarin.Forms.Portable6.Services;

namespace Xamarin.Forms.Portable6.ViewModels
{
    public class PhonePageViewModel : BindableBase
    {

        private readonly IPhoneService _PhoneService;

        public PhonePageViewModel()
        {
            _PhoneService = DependencyService.Get<IPhoneService>();
        }

        private string _Number;
        public string Number
        {
            get
            {
                return this._Number;
            }
            set
            {
                this.SetProperty(ref this._Number, value);
            }
        }

        private string _DisplayName;
        public string DisplayName
        {
            get
            {
                return this._DisplayName;
            }
            set
            {
                this.SetProperty(ref this._DisplayName, value);
            }
        }

        private ICommand _CallCommand;
        public ICommand CallCommand
        {
            get
            {
                return _CallCommand ?? new Microsoft.Practices.Prism.Commands.DelegateCommand(
                    () =>
                    {
                        this._PhoneService.ShowUI(this.Number, this.DisplayName);
                    },
                    () => true);
            }
        }

    }

}
