using System.Windows;
using System.Windows.Interactivity;

namespace WPF3.Behaviors
{

    public sealed class BooleanVisualStateBehavior : Behavior<FrameworkElement>
    {

        public static readonly DependencyProperty StateProperty = DependencyProperty.RegisterAttached(
            "State",
            typeof(bool),
            typeof(BooleanVisualStateBehavior),
            new PropertyMetadata(false, PropertyChangedCallback));

        private static void PropertyChangedCallback(DependencyObject dependencyObject, DependencyPropertyChangedEventArgs dependencyPropertyChangedEventArgs)
        {
            var visualStateSettingBehavior = dependencyObject as BooleanVisualStateBehavior;
            if (visualStateSettingBehavior == null)
            {
                return;
            }

            var state = visualStateSettingBehavior.State
                ? visualStateSettingBehavior.TrueState
                : visualStateSettingBehavior.FalseState;
            bool result;
            var frameworkElement = visualStateSettingBehavior.AssociatedObject;
            result = VisualStateManager.GoToElementState(frameworkElement, state, true);
        }

        public bool State
        {
            get
            {
                return (bool)this.GetValue(StateProperty);
            }
            set
            {
                this.SetValue(StateProperty, value);
            }
        }

        public string TrueState
        {
            get;
            set;
        }

        public string FalseState
        {
            get;
            set;
        }

    }

}
