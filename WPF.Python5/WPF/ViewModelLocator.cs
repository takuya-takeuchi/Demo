/*
  In App.xaml:
  <Application.Resources>
      <vm:ViewModelLocator xmlns:vm="clr-namespace:WPF1"
                                   x:Key="Locator" />
  </Application.Resources>
  
  In the View:
  DataContext="{Binding Source={StaticResource Locator}, Path=ViewModelName}"
*/

using System;
using System.ServiceModel;
using System.ServiceModel.Web;
using System.Windows;
using GalaSoft.MvvmLight;
using GalaSoft.MvvmLight.Ioc;
using Microsoft.Practices.ServiceLocation;
using WPFPython.Contracts.Service;
using WPFPython.ViewModels;
using WPFPython.ViewModels.Interfaces;

namespace WPFPython
{
    /// <summary>
    /// This class contains static references to all the view models in the
    /// application and provides an entry point for the bindings.
    /// <para>
    /// See http://www.galasoft.ch/mvvm
    /// </para>
    /// </summary>
    public class ViewModelLocator
    {

        private static readonly ServiceHost _ServiceHost;

        static ViewModelLocator()
        {
            ServiceLocator.SetLocatorProvider(() => SimpleIoc.Default);

            if (ViewModelBase.IsInDesignModeStatic)
            {
                // SimpleIoc.Default.Register<IDataService, Design.DesignDataService>();
            }
            else
            {
                // SimpleIoc.Default.Register<IDataService, DataService>();
            }
            
            var port = 5002;
            var baseAddress = "http://" + Environment.MachineName + $":{port}/{typeof(DateTimeService).Name}";
            var dateTimeService = new DateTimeService();
            _ServiceHost = new WebServiceHost(dateTimeService, new Uri(baseAddress));
            _ServiceHost.Open();

            SimpleIoc.Default.Register<IDateTimeService>(() => dateTimeService);
            SimpleIoc.Default.Register<IMainViewModel, MainViewModel>();

            Application.Current.Exit += CurrentOnExit;
        }

        private static void CurrentOnExit(object sender, ExitEventArgs exitEventArgs)
        {
            try
            {
                _ServiceHost.Close();
            }
            catch (Exception e)
            {
            }
        }

        /// <summary>
        /// Gets the Main property.
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance",
            "CA1822:MarkMembersAsStatic",
            Justification = "This non-static member is needed for data binding purposes.")]
        public IMainViewModel Main
        {
            get
            {
                return ServiceLocator.Current.GetInstance<IMainViewModel>();
            }
        }
    }
}