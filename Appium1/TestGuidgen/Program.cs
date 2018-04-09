using System;
using OpenQA.Selenium.Remote;

namespace TestGuidgen
{

    internal class Program
    {

        static void Main()
        {
            //Windows Application Driver実行
            var serverPath = System.IO.Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.ProgramFilesX86),
                @"Windows Application Driver",
                "WinAppDriver.exe"
            );
            System.Diagnostics.Process.Start(serverPath);

            // Guidgen操作
            var appCapabilities = new DesiredCapabilities();
            appCapabilities.SetCapability("app", @"C:\Program Files (x86)\Microsoft Visual Studio 14.0\Common7\Tools\guidgen.exe");

            var driver = new RemoteWebDriver(new Uri(@"http://127.0.0.1:4723"), appCapabilities);
            driver.Manage().Timeouts().ImplicitlyWait(TimeSpan.FromSeconds(2));

            // 最大 5 秒まで待機してボタン押下を繰り返す
            for (var i = 0; i < 10; i++)
            {
                driver.FindElementByName("新規 GUID(N)").Click();
                driver.Manage().Timeouts().ImplicitlyWait(TimeSpan.FromSeconds(5));
            }
            
            driver.FindElementByName("終了(X)").Click();

            Console.Write("Press any key to continue . . . ");
            Console.ReadKey(true);
        }

    }

}
