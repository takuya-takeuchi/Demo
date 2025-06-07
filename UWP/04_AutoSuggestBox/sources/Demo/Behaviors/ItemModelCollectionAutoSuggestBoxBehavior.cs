using System;
using System.Collections.Generic;
using System.Linq;
using Demo.Models;

namespace Demo.Behaviors
{

    internal sealed class ItemModelCollectionAutoSuggestBoxBehavior : AutoSuggestBoxBehaviorBase<ItemModel>
    {

        #region Properties

        protected override string TextMemberPath => nameof(ItemModel.Value);

        #endregion

        #region Methods

        #region Overrides

        protected override string GetText(ItemModel chosenSuggestion)
        {
            return chosenSuggestion.Value;
        }

        protected override IReadOnlyCollection<ItemModel> OnFiltered(IEnumerable<ItemModel> itemsSource, string queryText)
        {
            return itemsSource.Where(x => x.Value.IndexOf(queryText, StringComparison.OrdinalIgnoreCase) >= 0).ToArray();
        }

        #endregion

        #endregion

    }

}
