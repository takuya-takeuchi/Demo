using System;
using System.Windows;

namespace Demo
{
    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : Application
    {

        #region Methods

        [STAThread]
        public static void Main()
        {
            // you can do something before process start

            var shell = new Shell();
            shell.Run();
        }

        #endregion

    }

}
