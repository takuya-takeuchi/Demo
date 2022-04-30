namespace WPF.Models
{

    public sealed class GridSizeModel
    {

        #region Constructors

        public GridSizeModel(int column , int row)
        {
            this._Column = column;
            this._Row = row;
        }

        #endregion

        #region Properties

        private int _Column;

        public int Column => this._Column;

        private int _Row;

        public int Row => this._Row;

        #endregion

        #region Methods

        #region Overrids

        public override string ToString()
        {
            return $"{this._Row}x{this._Column}";
        }

        #endregion

        #endregion

    }

}
