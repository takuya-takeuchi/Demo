using System;
using System.Runtime.InteropServices;

// ReSharper disable once CheckNamespace
namespace Demo.PInvoke
{

    internal sealed partial class NativeMethods
    {

        #region Fields

        private const string NativeLibrary = "libLuhnc.dylib";

        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool luhn_validateString(byte[] number, uint type);

        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint luhn_typeFromString(byte[] number);

        #endregion

    }

}
