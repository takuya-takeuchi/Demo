using Demo.Models;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Demo.Behaviors
{

    internal sealed class KeyValuePairCollectionAutoSuggestBoxBehavior : AutoSuggestBoxBehaviorBase<KeyValuePair<string, string>>
    {

        #region Properties

        protected override string TextMemberPath => nameof(KeyValuePair<string, string>.Value);

        #endregion

        #region Methods

        #region Overrides

        protected override string GetText(KeyValuePair<string, string> chosenSuggestion)
        {
            return chosenSuggestion.Value;
        }

        protected override IReadOnlyCollection<KeyValuePair<string, string>> OnFiltered(IEnumerable<KeyValuePair<string, string>> itemsSource, string queryText)
        {
            return itemsSource.Where(x => x.Value.IndexOf(queryText, StringComparison.OrdinalIgnoreCase) >= 0).ToArray();
        }

        #endregion

        #endregion

    }

}
