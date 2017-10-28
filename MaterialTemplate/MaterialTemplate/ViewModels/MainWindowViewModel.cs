using System;
using MaterialTemplate.Services.Interfaces;
using MaterialTemplate.ViewModels.Interfaces;
using Prism.Mvvm;

namespace MaterialTemplate.ViewModels
{

    public sealed class MainWindowViewModel : BindableBase, IMainWindowViewModel
    {

        #region Events
        #endregion

        #region Fields

        private readonly ILoggerService _LoggerService;

        #endregion

        #region Constructors

        public MainWindowViewModel(ILoggerService loggerService)
        {
            this._LoggerService = loggerService ?? throw new ArgumentNullException(nameof(loggerService));
        }

        #endregion

        #region Properties
        #endregion

        #region Methods

        #region Overrids
        #endregion

        #region Event Handlers
        #endregion

        #region Helpers
        #endregion

        #endregion

        #region Implements IMainWindowViewModel

        public string Title
        {
            get
            {
                return "Material Design Project Template";
            }
        }

        #endregion

    }

}
