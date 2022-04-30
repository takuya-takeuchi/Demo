using System.Text.RegularExpressions;
using System.Windows;
using System.Windows.Input;
using Windows.UI.Xaml;

using Microsoft.Toolkit.Uwp.UI.Controls;
using Microsoft.Xaml.Interactivity;
using Prism.Commands;
using UWP.ShortCutKey.Services;

namespace UWP.ShortCutKey.Behaviors
{

    //public sealed class ShortcutKeyBehaviors : Behavior<DependencyObject>
    //{

    //    #region Dependency Properties


    //    public static readonly DependencyProperty KeyCommandProperty = DependencyProperty.Register(nameof(KeyCommand),
    //                                                                                               typeof(DelegateCommand),
    //                                                                                               typeof(ShortcutKeyBehaviors),
    //                                                                                               new PropertyMetadata(null));

    //    public DelegateCommand KeyCommand
    //    {
    //        get => (DelegateCommand)GetValue(KeyCommandProperty);
    //        set => SetValue(KeyCommandProperty, value);
    //    }

    //    public static readonly DependencyProperty KeyNameProperty = DependencyProperty.Register(nameof(KeyName),
    //                                                                                            typeof(string),
    //                                                                                            typeof(ShortcutKeyBehaviors),
    //                                                                                            new PropertyMetadata(""));

    //    public string KeyName
    //    {
    //        get => (string)GetValue(KeyNameProperty);
    //        set => SetValue(KeyNameProperty, value);
    //    }

    //    #endregion

    //    #region Methods

    //    #region Overrides

    //    protected override void OnAttached()
    //    {
    //        base.OnAttached();

    //        ShortcutKeyManager.Instance.Add(this.KeyName, this.AssociatedObject, "", this.KeyCommand);
    //    }

    //    protected override void OnDetaching()
    //    {
    //        base.OnDetaching();
    //    }

    //    #endregion

    //    #endregion

    //}


    public sealed class ShortcutKeyBehaviors
    {

        #region Dependency Properties


        public static readonly DependencyProperty KeyCommandProperty = DependencyProperty.RegisterAttached("KeyCommand",
                                                                                                           typeof(DelegateCommand),
                                                                                                           typeof(ShortcutKeyBehaviors),
                                                                                                           new PropertyMetadata(null, PropertyChangedCallback));
        
        public static DelegateCommand GetKeyCommand(DependencyObject obj)
        {
            return (DelegateCommand)obj.GetValue(KeyCommandProperty);
        }

        public static void SetKeyCommand(DependencyObject obj, DelegateCommand value)
        {
            obj.SetValue(KeyCommandProperty, value);
        }

        public static readonly DependencyProperty KeyNameProperty = DependencyProperty.RegisterAttached("KeyName",
                                                                                                        typeof(string),
                                                                                                        typeof(ShortcutKeyBehaviors),
                                                                                                        new PropertyMetadata("", PropertyChangedCallback));

        public static string GetKeyName(DependencyObject obj)
        {
            return (string)obj.GetValue(KeyNameProperty);
        }

        public static void SetKeyName(DependencyObject obj, string value)
        {
            obj.SetValue(KeyNameProperty, value);
        }

        #endregion

        #region Methods

        #region Event Handlers

        private static void PropertyChangedCallback(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            var keyName = ShortcutKeyBehaviors.GetKeyName(d);
            var command = ShortcutKeyBehaviors.GetKeyCommand(d);
            if (!string.IsNullOrWhiteSpace(keyName) && command != null)
                ShortcutKeyManager.Instance.Add(keyName, d, "", command);
        }

        #endregion

        #endregion

    }

}
