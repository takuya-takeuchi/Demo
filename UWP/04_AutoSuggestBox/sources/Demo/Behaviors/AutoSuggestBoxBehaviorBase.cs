using System.Collections.Generic;
using System.Linq;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Microsoft.Xaml.Interactivity;

namespace Demo.Behaviors
{

    internal abstract class AutoSuggestBoxBehaviorBase<T> : Behavior<AutoSuggestBox>
    {

        #region Properties

        public object ItemsSource
        {
            get => GetValue(ItemsSourceProperty);
            set => SetValue(ItemsSourceProperty, value);
        }

        public static readonly DependencyProperty ItemsSourceProperty =
            DependencyProperty.Register(
                nameof(ItemsSource),
                typeof(object),
                typeof(AutoSuggestBoxBehaviorBase<T>),
                new PropertyMetadata(null));

        protected abstract string TextMemberPath
        {
            get;
        }

        #endregion

        #region Methods

        #region Overrides

        protected override void OnAttached()
        {
            base.OnAttached();

            this.AssociatedObject.TextMemberPath = this.TextMemberPath ?? "";

            this.AssociatedObject.GotFocus += this.OnGotFocus;
            this.AssociatedObject.QuerySubmitted += this.OnQuerySubmitted;
            this.AssociatedObject.SuggestionChosen += this.OnSuggestionChosen;
            this.AssociatedObject.TextChanged += this.OnTextChanged;
        }

        protected override void OnDetaching()
        {
            base.OnDetaching();

            this.AssociatedObject.GotFocus -= this.OnGotFocus;
            this.AssociatedObject.QuerySubmitted -= this.OnQuerySubmitted;
            this.AssociatedObject.SuggestionChosen -= this.OnSuggestionChosen;
            this.AssociatedObject.TextChanged -= this.OnTextChanged;
        }

        protected abstract string GetText(T chosenSuggestion);

        protected abstract IReadOnlyCollection<T> OnFiltered(IEnumerable<T> itemSource, string queryTex);

        #endregion

        #region Event Handlers

        private void OnGotFocus(object sender, RoutedEventArgs e)
        {
            if (!(sender is AutoSuggestBox box))
                return;
            if (!string.IsNullOrEmpty(box.Text)) 
                return;

            box.ItemsSource = this.ItemsSource;
            box.IsSuggestionListOpen = true;
        }

        private void OnQuerySubmitted(AutoSuggestBox sender, AutoSuggestBoxQuerySubmittedEventArgs args)
        {
        }

        private void OnSuggestionChosen(AutoSuggestBox sender, AutoSuggestBoxSuggestionChosenEventArgs args)
        {
        }

        private void OnTextChanged(AutoSuggestBox sender, AutoSuggestBoxTextChangedEventArgs args)
        {
            if (args.Reason != AutoSuggestionBoxTextChangeReason.UserInput)
                return;

            if (!(this.ItemsSource is object[] items))
                return;

            var itemSource = items.Cast<T>();
            sender.ItemsSource = this.OnFiltered(itemSource, sender.Text);
        }

        #endregion

        #endregion

    }

}
