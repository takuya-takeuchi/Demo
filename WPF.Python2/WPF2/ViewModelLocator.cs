/*
  In App.xaml:
  <Application.Resources>
      <vm:ViewModelLocator xmlns:vm="clr-namespace:WPF1"
                                   x:Key="Locator" />
  </Application.Resources>
  
  In the View:
  DataContext="{Binding Source={StaticResource Locator}, Path=ViewModelName}"
*/

using System.Windows;
using GalaSoft.MvvmLight;
using GalaSoft.MvvmLight.Ioc;
using IronPython.Hosting;
using Microsoft.Practices.ServiceLocation;
using WPFPython2.ViewModels;
using WPFPython2.ViewModels.Interfaces;

namespace WPFPython2
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

            SimpleIoc.Default.Register<IMessageDialog>(() => new MessageDialog(Application.Current.MainWindow));

            var py = Python.CreateRuntime();
            SimpleIoc.Default.Register<IPythonWrapper>(()=> new PythonWrapper(py, @"pythons\Python2.py"));
            SimpleIoc.Default.Register<IMainViewModel, MainViewModel>();
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