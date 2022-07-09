using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using OpenQA.Selenium.Appium;
using OpenQA.Selenium.Appium.Windows;
using OpenQA.Selenium.Remote;

namespace Demo.Test
{

    [TestClass]
    public sealed class MainPageViewModel
    {

        #region Fields

        private const string AppDriverUrl = "http://localhost:4724";
        
        private const string DemoAppId = @"Demo_zz8mjkem88v5c!App";

        private static WindowsDriver<AppiumWebElement> AppSession;

        private const string LeftOperand = "LeftOperand";

        private const string RightOperand = "RightOperand";

        private const string Result = "Result";

        private const string Operator = "Operator";

        private const string AddCommand = "AddCommand";

        private const string SubtractCommand = "SubtractCommand";

        private const string MultiplyCommand = "MultiplyCommand";

        private const string DivideCommand = "DivideCommand";

        #endregion

        #region Methods

        [ClassInitialize]
        public static void Setup(TestContext context)
        {
            var appiumOptions = new AppiumOptions();
            appiumOptions.AddAdditionalCapability("app", DemoAppId);
            appiumOptions.AddAdditionalCapability("deviceName", "WindowsPC");

            AppSession = new WindowsDriver<AppiumWebElement>(new Uri(AppDriverUrl), appiumOptions);
            Assert.IsNotNull(AppSession);
        }

        [ClassCleanup]
        public static void TestsCleanup()
        {
            AppSession?.Dispose();
            AppSession = null;
        }

        #endregion

        [TestMethod]
        public void Add()
        {
            AppSession.FindElementByName(LeftOperand).Clear();
            AppSession.FindElementByName(LeftOperand).SendKeys("2");
            AppSession.Manage().Timeouts().ImplicitWait = TimeSpan.FromSeconds(1);

            AppSession.FindElementByName(RightOperand).Clear();
            AppSession.FindElementByName(RightOperand).SendKeys("5");
            AppSession.Manage().Timeouts().ImplicitWait = TimeSpan.FromSeconds(1);

            AppSession.FindElementByName(AddCommand).Click();

            var txtResultTextElement = AppSession.FindElementByName(Result) as RemoteWebElement;
            Assert.IsNotNull(txtResultTextElement);
            Assert.AreEqual("7", txtResultTextElement.Text);

            txtResultTextElement = AppSession.FindElementByName(Operator) as RemoteWebElement;
            Assert.IsNotNull(txtResultTextElement);
            Assert.AreEqual("+", txtResultTextElement.Text);
        }

        [TestMethod]
        public void Subtract()
        {
            AppSession.FindElementByName(LeftOperand).Clear();
            AppSession.FindElementByName(LeftOperand).SendKeys("1");
            AppSession.FindElementByName(LeftOperand).SendKeys("2");
            AppSession.Manage().Timeouts().ImplicitWait = TimeSpan.FromSeconds(1);

            AppSession.FindElementByName(RightOperand).Clear();
            AppSession.FindElementByName(RightOperand).SendKeys("5");
            AppSession.Manage().Timeouts().ImplicitWait = TimeSpan.FromSeconds(1);

            AppSession.FindElementByName(SubtractCommand).Click();

            var txtResultTextElement = AppSession.FindElementByName(Result) as RemoteWebElement;
            Assert.IsNotNull(txtResultTextElement);
            Assert.AreEqual("7", txtResultTextElement.Text);

            txtResultTextElement = AppSession.FindElementByName(Operator) as RemoteWebElement;
            Assert.IsNotNull(txtResultTextElement);
            Assert.AreEqual("-", txtResultTextElement.Text);
        }

        [TestMethod]
        public void Multiply()
        {
            AppSession.FindElementByName(LeftOperand).Clear();
            AppSession.FindElementByName(LeftOperand).SendKeys("7");
            AppSession.Manage().Timeouts().ImplicitWait = TimeSpan.FromSeconds(1);

            AppSession.FindElementByName(RightOperand).Clear();
            AppSession.FindElementByName(RightOperand).SendKeys("4");
            AppSession.Manage().Timeouts().ImplicitWait = TimeSpan.FromSeconds(1);

            AppSession.FindElementByName(MultiplyCommand).Click();

            var txtResultTextElement = AppSession.FindElementByName(Result) as RemoteWebElement;
            Assert.IsNotNull(txtResultTextElement);
            Assert.AreEqual("28", txtResultTextElement.Text);

            txtResultTextElement = AppSession.FindElementByName(Operator) as RemoteWebElement;
            Assert.IsNotNull(txtResultTextElement);
            Assert.AreEqual("*", txtResultTextElement.Text);
        }

        [TestMethod]
        public void Divide()
        {
            AppSession.FindElementByName(LeftOperand).Clear();
            AppSession.FindElementByName(LeftOperand).SendKeys("9");
            AppSession.Manage().Timeouts().ImplicitWait = TimeSpan.FromSeconds(1);

            AppSession.FindElementByName(RightOperand).Clear();
            AppSession.FindElementByName(RightOperand).SendKeys("4");
            AppSession.Manage().Timeouts().ImplicitWait = TimeSpan.FromSeconds(1);

            AppSession.FindElementByName(DivideCommand).Click();

            var txtResultTextElement = AppSession.FindElementByName(Result) as RemoteWebElement;
            Assert.IsNotNull(txtResultTextElement);
            Assert.AreEqual("2", txtResultTextElement.Text);

            txtResultTextElement = AppSession.FindElementByName(Operator) as RemoteWebElement;
            Assert.IsNotNull(txtResultTextElement);
            Assert.AreEqual("/", txtResultTextElement.Text);
        }

    }

}
