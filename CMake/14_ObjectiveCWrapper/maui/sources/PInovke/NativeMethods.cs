using System;
using System.Runtime.InteropServices;

// ReSharper disable once CheckNamespace
namespace Demo.PInvoke
{

    internal sealed partial class NativeMethods
    {

        #region Fields

        private const string NativeLibrary = "Luhnc";

        [DllImport(NativeLibrary)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool luhn_validateString(byte[] str, uint type);

        #endregion

    }

}
