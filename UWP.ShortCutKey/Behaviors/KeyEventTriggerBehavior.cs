using System.Text.RegularExpressions;
using System.Windows;
using System.Windows.Input;
using Windows.System;
using Windows.UI.Core;
using Windows.UI.Xaml;

using Microsoft.Toolkit.Uwp.UI.Controls;
using Microsoft.Xaml.Interactivity;
using Prism.Commands;
using UWP.ShortCutKey.Services;

namespace UWP.ShortCutKey.Behaviors
{

    public class KeyEventTriggerBehavior : Behavior<DependencyObject>
    {

        #region Dependency Properties

        public static readonly DependencyProperty GroupNameProperty = DependencyProperty.Register(nameof(GroupName),
                                                                                                  typeof(string),
                                                                                                  typeof(ShortcutKeyBehaviors),
                                                                                                  new PropertyMetadata(""));

        public string GroupName
        {
            get => (string)GetValue(GroupNameProperty);
            set => SetValue(GroupNameProperty, value);
        }

        #endregion

        #region Methods

        #region Overrides

        protected override void OnAttached()
        {
            this.Register();
            base.OnAttached();
        }

        protected override void OnDetaching()
        {
            this.Unregister();
            base.OnDetaching();
        }

        #region Helpers

        private void Register()
        {
            var frameworkElement = this.AssociatedObject as FrameworkElement;
            if (frameworkElement == null)
                return;

            frameworkElement.Unloaded -= this.FrameworkElementUnloaded;
            frameworkElement.Unloaded += this.FrameworkElementUnloaded;

            this.Dispatcher.AcceleratorKeyActivated += this.DispatcherAccelaratorKeyActivated;
        }

        private void Unregister()
        {
            var frameworkElement = this.AssociatedObject as FrameworkElement;
            if (frameworkElement == null) 
                return;

            frameworkElement.Unloaded -= this.FrameworkElementUnloaded;

            this.Dispatcher.AcceleratorKeyActivated -= this.DispatcherAccelaratorKeyActivated;
        }

        private void DispatcherAccelaratorKeyActivated(CoreDispatcher sender, AcceleratorKeyEventArgs args)
        {
            if (args.KeyStatus.RepeatCount != 1)
                return;

            if (args.EventType != CoreAcceleratorKeyEventType.KeyDown)
                return;

            var shift = (Window.Current.CoreWindow.GetKeyState(VirtualKey.Shift) & CoreVirtualKeyStates.Down) == CoreVirtualKeyStates.Down;
            var control = (Window.Current.CoreWindow.GetKeyState(VirtualKey.Control) & CoreVirtualKeyStates.Down) == CoreVirtualKeyStates.Down;
            var alt = (Window.Current.CoreWindow.GetKeyState(VirtualKey.Menu) & CoreVirtualKeyStates.Down) == CoreVirtualKeyStates.Down;


            if (ShortcutKeyManager.Instance.Execute(this.GroupName, args.VirtualKey, control, shift, alt))
            {
                args.Handled = true;
            }
            else
            {
                args.Handled = false;
            }
            //if (this.Key == args.VirtualKey)
            //{
            //    Interaction.ExecuteActions(this, this.Actions, args);
            //    args.Handled = true;
            //}
        }

        private void FrameworkElementUnloaded(object sender, RoutedEventArgs e)
        {
            this.Unregister();
        }

        #endregion

        #endregion

        #endregion

    }

}
