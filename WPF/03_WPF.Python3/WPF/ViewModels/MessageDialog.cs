using System;
using System.Windows;
using WPFPython.ViewModels.Interfaces;

namespace WPFPython.ViewModels
{

    public sealed class MessageDialog : IMessageDialog
    {

        #region フィールド

        private readonly Window _Owner;

        #endregion

        #region コンストラクタ

        public MessageDialog(Window owner)
        {
            if (owner == null)
                throw new ArgumentNullException(nameof(owner));

            this._Owner = owner;
        }

        #endregion

        #region プロパティ
        #endregion

        #region メソッド

        public void ShowMessage(string message)
        {
            System.Windows.MessageBox.Show(this._Owner, message);
        }

        #endregion

    }

}