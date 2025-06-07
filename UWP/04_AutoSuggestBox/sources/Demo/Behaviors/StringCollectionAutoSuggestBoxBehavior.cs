using System;
using System.Collections.Generic;
using System.Linq;

namespace Demo.Behaviors
{

    internal sealed class StringCollectionAutoSuggestBoxBehavior : AutoSuggestBoxBehaviorBase<string>
    {

        #region Properties

        protected override string TextMemberPath => null;

        #endregion

        #region Methods

        #region Overrides

        protected override string GetText(string chosenSuggestion)
        {
            return chosenSuggestion;
        }

        protected override IReadOnlyCollection<string> OnFiltered(IEnumerable<string> itemsSource, string queryText)
        {
            return itemsSource.Where(x => x.IndexOf(queryText, StringComparison.OrdinalIgnoreCase) >= 0).ToArray();
        }

        #endregion

        #endregion

    }

}
