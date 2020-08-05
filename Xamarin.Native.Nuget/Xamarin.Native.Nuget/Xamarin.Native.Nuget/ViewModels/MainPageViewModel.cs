using System.Windows.Input;
using Prism.Commands;
using Prism.Navigation;
using Xamarin.Native.Nuget.Services.Interfaces;
using Xamarin.Native.Nuget.ViewModels.Interfaces;

namespace Xamarin.Native.Nuget.ViewModels
{

    public class MainPageViewModel : ViewModelBase,  IMainPageViewModel
    {

        #region Fields

        private readonly INativeService _NativeService;

        #endregion

        #region Constructors

        public MainPageViewModel(INavigationService navigationService,
                                 INativeService nativeService)
            : base(navigationService)
        {
            this._NativeService = nativeService;
            this.Title = "Main Page";
        }

        #endregion

        #region Properties

        private string _Title;

        public string Title
        {
            get => this._Title;
            private set => this.SetProperty(ref this._Title, value);
        }

        private int _Left;

        public int Left
        {
            get => this._Left;
            set => this.SetProperty(ref this._Left, value);
        }

        private int _Right;

        public int Right
        {
            get => this._Right;
            set => this.SetProperty(ref this._Right, value);
        }

        private int _AddResult;

        public int AddResult
        {
            get => this._AddResult;
            private set => this.SetProperty(ref this._AddResult, value);
        }

        private int _MulResult;

        public int MulResult
        {
            get => this._MulResult;
            private set => this.SetProperty(ref this._MulResult, value);
        }

        private ICommand _CalcCommand;

        public ICommand CalcCommand
        {
            get
            {
                return this._CalcCommand ?? (this._CalcCommand = new DelegateCommand(() =>
                {
                    this.AddResult = this._NativeService.Add(this._Left, this._Right);
                }));

            }
        }

        #endregion

    }

}