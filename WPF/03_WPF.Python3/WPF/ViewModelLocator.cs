/*
  In App.xaml:
  <Application.Resources>
      <vm:ViewModelLocator xmlns:vm="clr-namespace:WPF1"
                                   x:Key="Locator" />
  </Application.Resources>
  
  In the View:
  DataContext="{Binding Source={StaticResource Locator}, Path=ViewModelName}"
*/

using System.Collections.Generic;
using System.Windows;
using GalaSoft.MvvmLight;
using GalaSoft.MvvmLight.Ioc;
using IronPython.Hosting;
using Microsoft.Practices.ServiceLocation;
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

            // https://mail.python.org/pipermail/ironpython-users/2013-January/016388.html
            var options = new Dictionary<string, object>();
            options["Frames"] = true;
            options["FullFrames"] = true;
            var py = Python.CreateRuntime(options);

            dynamic sys = py.GetSysModule();
            sys.path.append(@"C:\Program Files (x86)\IronPython 2.7");
            sys.path.append(@"C:\Program Files (x86)\IronPython 2.7\DLLs");
            sys.path.append(@"C:\Program Files (x86)\IronPython 2.7\Lib");
            sys.path.append(@"C:\Program Files (x86)\IronPython 2.7\Lib\site-packages");

            SimpleIoc.Default.Register<IPythonWrapper>(() => new PythonWrapper(py, @"pythons\Python.py"));
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