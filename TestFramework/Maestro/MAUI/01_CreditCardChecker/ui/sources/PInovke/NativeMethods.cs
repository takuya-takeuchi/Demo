using System;
using System.Runtime.InteropServices;

// ReSharper disable once CheckNamespace
namespace Demo.PInvoke
{

    internal sealed partial class NativeMethods
    {

        #region Fields

        private const string NativeLibrary = "Luhn";

        // [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl)]
        [DllImport(NativeLibrary)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool luhn_validateString(byte[] number, uint length);

        #endregion

    }

}
