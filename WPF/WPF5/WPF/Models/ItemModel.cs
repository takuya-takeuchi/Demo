namespace WPF.Models
{

    public sealed class ItemModel
    {

        #region Constructors

        public ItemModel(int number)
        {
            this._Number = number;
        }

        #endregion

        #region Properties

        private int _Number;

        public int Number => this._Number;

        #endregion

    }

}
