using System;
using Prism;
using Prism.Ioc;
using Xamarin.Forms;
using Xamarin.Forms.Platform.GTK;

namespace Sample.Gtk
{

    public class Program
    {

        [STAThread]
        public static void Main(string[] args)
        {
            global::Gtk.Application.Init();
            Forms.Init();

            var app = new App(new GtkInitializer());
            var window = new FormsWindow();
            window.LoadApplication(app);
            window.SetApplicationTitle("Your App Name");
            window.Show();

            global::Gtk.Application.Run();
        }
    }

    public class GtkInitializer : IPlatformInitializer
    {
        public void RegisterTypes(IContainerRegistry containerRegistry)
        {
            // Register any platform specific implementations
        }
    }

}