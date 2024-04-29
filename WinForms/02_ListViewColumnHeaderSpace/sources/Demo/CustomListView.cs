using System;
using System.Runtime.InteropServices;
using System.Windows.Forms;

namespace Demo
{

    internal sealed class CustomListView : ListView
    {

        #region P/Invoke

        [DllImport("user32.dll", EntryPoint = "GetWindowLong", CharSet = CharSet.Auto)]
        private static extern IntPtr GetWindowLong32(IntPtr hWnd, int nIndex);

        [DllImport("user32.dll", EntryPoint = "GetWindowLongPtr", CharSet = CharSet.Auto)]
        private static extern IntPtr GetWindowLongPtr64(IntPtr hWnd, int nIndex);

        [DllImport("user32.dll", EntryPoint = "SetWindowLong", CharSet = CharSet.Auto)]
        private static extern IntPtr SetWindowLongPtr32(IntPtr hWnd, int nIndex, int dwNewLong);

        [DllImport("user32.dll", EntryPoint = "SetWindowLongPtr", CharSet = CharSet.Auto)]
        private static extern IntPtr SetWindowLongPtr64(IntPtr hWnd, int nIndex, int dwNewLong);

        private const int GWL_STYLE = -16;

        private const int WS_VSCROLL = 0x00200000;

        private const int WS_HSCROLL = 0x00100000;

        private const int WM_NCCALCSIZE = 0x83;

        #endregion

        #region Fields

        private readonly int _VerticalScrollBarWidth = SystemInformation.VerticalScrollBarWidth;

        private const int ColumnExtraSpace = 4;

        private bool _IsVerticalScrollBarVisible = false;

        private bool _IsHorizontalScrollBarVisible = false;

        #endregion

        #region Constructors

        public CustomListView()
        {
            this.DoubleBuffered = true;
        }

        #endregion

        #region Methods

        #region Overrides

        protected override void OnColumnReordered(ColumnReorderedEventArgs e)
        {
            var listView = this;
            var columns = listView.Columns;
            var columnsCount = columns.Count;
            var mostRightDisplayColumnIndex = columnsCount - 1;

            // Most right columns did not move and moved column does not go to most right
            if (!(e.OldDisplayIndex == mostRightDisplayColumnIndex || e.NewDisplayIndex == mostRightDisplayColumnIndex))
                return;

            var nextMostRightColumnDisplayIndex = e.OldDisplayIndex == mostRightDisplayColumnIndex ? e.NewDisplayIndex : e.OldDisplayIndex;
            var mostRightColumn = FindColumnWithDisplayIndex(nextMostRightColumnDisplayIndex);
            if (mostRightColumn == null)
                return;

            this.FillColumn(mostRightColumn);
        }
        
        protected override void OnSizeChanged(EventArgs e)
        {
            var listView = this;
            var columns = listView.Columns;
            var columnsCount = columns.Count;
            var mostRightDisplayColumnIndex = columnsCount - 1;

            var mostRightColumn = FindColumnWithDisplayIndex(mostRightDisplayColumnIndex);
            if (mostRightColumn == null)
                return;

            this.FillColumn(mostRightColumn);
        }

        protected override void OnColumnWidthChanging(ColumnWidthChangingEventArgs e)
        {
            // Not allow change size by double-click on border of column headers
            var listView = this;
            var columns = listView.Columns;
            var column = columns[e.ColumnIndex];

            e.NewWidth = column.Width;
            e.Cancel = true;
        }

        protected override void OnColumnWidthChanged(ColumnWidthChangedEventArgs e)
        {
            var listView = this;
            var columns = listView.Columns;
            var columnsCount = columns.Count;
            var mostRightDisplayColumnIndex = columnsCount - 1;

            var mostRightColumn = FindColumnWithDisplayIndex(mostRightDisplayColumnIndex);
            if (mostRightColumn == null)
                return;

            this.FillColumn(mostRightColumn);
        }

        protected override void WndProc(ref Message m)
        {
            switch (m.Msg)
            {
                case WM_NCCALCSIZE:
                    var style = GetWindowLong(this.Handle, GWL_STYLE);

                    var updateSize = false;
                    var isVerticalScrollBarVisible = (style & WS_VSCROLL) == WS_VSCROLL;
                    if (isVerticalScrollBarVisible != this._IsVerticalScrollBarVisible)
                    {
                        this._IsVerticalScrollBarVisible = isVerticalScrollBarVisible;
                        updateSize = true;
                    }
                    var isHorizontalScrollBarVisible = (style & WS_HSCROLL) == WS_HSCROLL;
                    if (isHorizontalScrollBarVisible != this._IsHorizontalScrollBarVisible)
                    {
                        this._IsHorizontalScrollBarVisible = isHorizontalScrollBarVisible;
                        updateSize = true;
                    }

                    if (updateSize)
                        this.OnColumnWidthChanged(new ColumnWidthChangedEventArgs(0));

                    base.WndProc(ref m);
                    break;
                default:
                    base.WndProc(ref m);
                    break;
            }
        }

        #endregion

        #region Helpers

        private void FillColumn(ColumnHeader mostRightDisplayColumn)
        {
            var listView = this;
            var columns = listView.Columns;
            var columnsCount = columns.Count;

            var width = 0;
            for (var index = 0; index < columnsCount; index++)
            {
                var column = listView.Columns[index];
                if (column == mostRightDisplayColumn)
                    continue;

                width += column.Width;
            }

            // new size is smaller than total column header width
            if (listView.Width <= (width + mostRightDisplayColumn.Width - ColumnExtraSpace))
                return;

            var newWidth = listView.Width - width - ColumnExtraSpace - (this._IsVerticalScrollBarVisible ? this._VerticalScrollBarWidth : 0);
            if (mostRightDisplayColumn.Width != newWidth)
                mostRightDisplayColumn.Width = newWidth;
        }

        private ColumnHeader FindColumnWithDisplayIndex(int displayIndex)
        {
            var listView = this;
            var columns = listView.Columns;
            var columnsCount = columns.Count;

            for (var index = 0; index < columnsCount; index++)
            {
                var column = listView.Columns[index];
                if (column.DisplayIndex == displayIndex)
                    return column;
            }

            return null;
        }

        private static int GetWindowLong(IntPtr hWnd, int nIndex)
        {
            return IntPtr.Size == 4 ? (int)GetWindowLong32(hWnd, nIndex) : (int)(long)GetWindowLongPtr64(hWnd, nIndex);
        }

        #endregion

        #endregion


    }

}