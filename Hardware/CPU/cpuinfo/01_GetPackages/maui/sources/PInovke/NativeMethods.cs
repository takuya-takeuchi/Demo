using System;
using System.Runtime.InteropServices;

// ReSharper disable once CheckNamespace
namespace Demo.PInvoke
{

    internal sealed partial class NativeMethods
    {

        #region Fields

        private const string NativeLibrary = "cpuinfo";

        [DllImport(NativeLibrary)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool cpuinfo_initialize();

        [DllImport(NativeLibrary)]
        public static extern uint cpuinfo_get_packages_count();

        [DllImport(NativeLibrary)]
        public static extern IntPtr cpuinfo_get_package(uint index);

        #endregion

    }

}
