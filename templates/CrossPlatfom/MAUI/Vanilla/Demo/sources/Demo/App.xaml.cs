using Microsoft.Maui.Controls;

namespace Demo
{

    public partial class App : Application
    {

        #region Constructors

        public App()
        {
            InitializeComponent();

            MainPage = new AppShell();
        }

        #endregion

    }

}
