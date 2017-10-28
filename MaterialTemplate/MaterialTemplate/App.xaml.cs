using System.Windows;
using Microsoft.Practices.Unity;

namespace MaterialTemplate
{
    /// <summary>
    /// App.xaml の相互作用ロジック
    /// </summary>

    public partial class App : Application
    {

        #region Constructors
        #endregion

        #region Properties
        
        private IUnityContainer Container { get; } = new UnityContainer();

        #endregion

        #region Methods

        #region Overrides

        protected override void OnStartup(StartupEventArgs e)
        {
            base.OnStartup(e);

            var bootstrapper = new Bootstrapper();
            bootstrapper.Run();
        }

        #endregion

        #region Event Handlers

        //private void ApplicationStartup(object sender, StartupEventArgs e)
        //{
        //    ViewModelLocationProvider.SetDefaultViewModelFactory(x => this.Container.Resolve(x));

        //    this.Container.Resolve<MainWindow>().Show();
        //}

        #endregion

        #endregion

    }
}
