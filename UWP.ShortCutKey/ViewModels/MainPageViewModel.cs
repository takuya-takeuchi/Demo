using Prism.Commands;
using Prism.Windows.Mvvm;
using UWP.ShortCutKey.ViewModels.Interfaces;

namespace UWP.ShortCutKey.ViewModels
{

    public sealed class MainPageViewModel : ViewModelBase, IMainPageViewModel
    {

        #region Constructors

        public MainPageViewModel()
        {
            this.TextMessage = "Initial Message";
        }

        #endregion

        #region Properties

        private string _TextMessage;

        public string TextMessage
        {
            get => this._TextMessage;
            set => this.SetProperty(ref this._TextMessage, value);
        }

        private DelegateCommand _SaveCommand;

        public DelegateCommand SaveCommand
        {
            get
            {
                return this._SaveCommand ?? (this._SaveCommand = new DelegateCommand(async () =>
                {
                    this.TextMessage = "Save Command was fired!!";
                }));
            }
        }

        private DelegateCommand _NewCommand;

        public DelegateCommand NewCommand
        {
            get
            {
                return this._NewCommand ?? (this._NewCommand = new DelegateCommand(async () =>
                {
                    this.TextMessage = "New Command was fired!!";
                }));
            }
        }

        #endregion

        #region Methods

        #region Overrids
        #endregion

        #region Event Handlers
        #endregion

        #region Helpers
        #endregion

        #endregion


    }

}