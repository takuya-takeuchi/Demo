using Prism.Mvvm;

using Demo.ViewModels.Interfaces;

namespace Demo.ViewModels
{

    internal sealed class MainWindowViewModel : BindableBase, IMainWindowViewModel
    {

        #region IMainWindowViewModel Members

        public string Message => nameof(MainWindowViewModel);

        #endregion

    }

}
