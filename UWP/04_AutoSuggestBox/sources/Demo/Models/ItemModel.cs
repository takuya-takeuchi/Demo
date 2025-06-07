namespace Demo.Models
{

    internal sealed class ItemModel
    {

        #region Constructors

        public ItemModel(string key, string value)
        {
            this.Key = key;
            this.Value = value;
        }

        #endregion

        #region Properties

        public string Key { get; }

        public string Value { get;}

        #endregion

    }

}
