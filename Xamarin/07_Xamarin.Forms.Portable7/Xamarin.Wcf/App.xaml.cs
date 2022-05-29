using System.ServiceModel;
using System.Windows;
using Microsoft.Practices.Unity;
using Prism.Mvvm;
using Xamarin.Wcf.Services;
using Xamarin.Wcf.ViewModels;
using Xamarin.Wcf.Views;

namespace Xamarin.Wcf
{
    /// <summary>
    /// App.xaml の相互作用ロジック
    /// </summary>
    public partial class App : Application
    {
        private ServiceHost _ServiceHost;

        public static UnityContainer Container;

        protected override void OnStartup(StartupEventArgs e)
        {
            base.OnStartup(e);

            var messageService = new MessageService();
            this._ServiceHost = new ServiceHost(messageService);
            this._ServiceHost.Open();

            var container = new UnityContainer();
            container.RegisterInstance(typeof(IMessageService), this._ServiceHost.SingletonInstance);

            ViewModelLocationProvider.Register(typeof(MainWindow).ToString(), () => new MainWindowViewModel(messageService));

            Container = container;
        }

        protected override void OnExit(ExitEventArgs e)
        {
            base.OnExit(e);

            this._ServiceHost.Close();
        }
    }
}
