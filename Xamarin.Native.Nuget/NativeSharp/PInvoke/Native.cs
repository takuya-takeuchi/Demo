using System.Runtime.InteropServices;

// ReSharper disable once CheckNamespace
namespace NativeSharp
{

    internal sealed partial class NativeMethods
    {

        [DllImport(NativeLibrary, CallingConvention = CallingConvention)]
        public static extern int native_add(int x, int y);

        [DllImport(NativeLibrary, CallingConvention = CallingConvention)]
        public static extern int native_mul(int x, int y);

    }

}