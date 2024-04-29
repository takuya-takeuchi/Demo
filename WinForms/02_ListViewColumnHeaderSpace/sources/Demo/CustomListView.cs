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

        private const int WM_NCCALCSIZE = 0x83;

        private readonly int _VerticalScrollBarWidth = SystemInformation.VerticalScrollBarWidth;

        #endregion

        #region Fields

        private bool _IsVerticalScrollBarVisible = false;

        #endregion

        #region Constructors

        public CustomListView()
        {
            this.DoubleBuffered = true;
        }

        #endregion

        #region Properties

        private int _MinimumMostRightColumnHeaderWidth = 200;

        public int MinimumMostRightColumnHeaderWidth
        {
            get => this._MinimumMostRightColumnHeaderWidth;
            set
            {
                this._MinimumMostRightColumnHeaderWidth = value;
            }
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

            ColumnHeader mostRightColumn = null;
            for (var index = 0; index < columnsCount; index++)
            {
                var column = listView.Columns[index];
                if (column.DisplayIndex != nextMostRightColumnDisplayIndex)
                    continue;

                mostRightColumn = column;
                break;
            }

            if (mostRightColumn == null)
                return;

            var width = 0;
            for (var index = 0; index < columnsCount; index++)
            {
                var column = listView.Columns[index];
                if (column.DisplayIndex == nextMostRightColumnDisplayIndex)
                    continue;

                width += column.Width;
            }

            width = listView.Width - width;
            mostRightColumn.Width = width - 4 - (this._IsVerticalScrollBarVisible ? this._VerticalScrollBarWidth : 0);
        }
        
        protected override void OnSizeChanged(EventArgs e)
        {
            var listView = this;
            var columns = listView.Columns;
            var columnsCount = columns.Count;
            var mostRightDisplayColumnIndex = columnsCount - 1;

            ColumnHeader mostRightColumn = null;
            for (var index = 0; index < columnsCount; index++)
            {
                var column = listView.Columns[index];
                if (column.DisplayIndex != mostRightDisplayColumnIndex)
                    continue;

                mostRightColumn = column;
                break;
            }

            if (mostRightColumn == null)
                return;

            var width = 0;
            for (var index = 0; index < columnsCount; index++)
            {
                var column = listView.Columns[index];
                if (column == mostRightColumn)
                    continue;

                width += column.Width;
            }

            // new size is smaller than total column header width
            if (listView.Width <= (width + mostRightColumn.Width - 4))
                return;

            width = listView.Width - width;
            mostRightColumn.Width = width - 4 - (this._IsVerticalScrollBarVisible ? this._VerticalScrollBarWidth : 0);
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

            var width = 0;
            for (var index = 0; index < columnsCount; index++)
            {
                var column = listView.Columns[index];
                if (column.DisplayIndex == mostRightDisplayColumnIndex)
                    continue;

                width += column.Width;
            }

            ColumnHeader mostRightColumn = null;
            for (var index = 0; index < columnsCount; index++)
            {
                var column = listView.Columns[index];
                if (column.DisplayIndex != mostRightDisplayColumnIndex)
                    continue;

                mostRightColumn = column;
                break;
            }

            if (mostRightColumn == null)
                return;

            // new size is smaller than total column header width
            //if (listView.Width <= (width + mostRightColumn.Width - 4))
            //    return;

            width = listView.Width - width;
            mostRightColumn.Width = width - 4 - (this._IsVerticalScrollBarVisible ? this._VerticalScrollBarWidth : 0);
        }

        protected override void WndProc(ref Message m)
        {
            switch (m.Msg)
            {
                case WM_NCCALCSIZE:
                    var style = GetWindowLong(this.Handle, GWL_STYLE);
                    var isVerticalScrollBarVisible = (style & WS_VSCROLL) == WS_VSCROLL;
                    if (isVerticalScrollBarVisible != this._IsVerticalScrollBarVisible)
                    {
                        this._IsVerticalScrollBarVisible = isVerticalScrollBarVisible;
                        this.OnColumnWidthChanged(new ColumnWidthChangedEventArgs(0));
                    }
                    base.WndProc(ref m);
                    break;
                default:
                    base.WndProc(ref m);
                    break;
            }
        }

        private static int GetWindowLong(IntPtr hWnd, int nIndex)
        {
            return IntPtr.Size == 4 ? (int)GetWindowLong32(hWnd, nIndex) : (int)(long)GetWindowLongPtr64(hWnd, nIndex);
        }

        private static int SetWindowLong(IntPtr hWnd, int nIndex, int dwNewLong)
        {
            return IntPtr.Size == 4 ? (int)SetWindowLongPtr32(hWnd, nIndex, dwNewLong) : (int)(long)SetWindowLongPtr64(hWnd, nIndex, dwNewLong);
        }

        #endregion

        #endregion


    }

}