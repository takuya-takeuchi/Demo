using System.Runtime.InteropServices;

// ReSharper disable once CheckNamespace
namespace NativeSharp
{

    internal sealed partial class NativeMethods
    {

        [DllImport(NativeLibrary, ExactSpelling = true, CallingConvention = CallingConvention)]
        public static extern int native_add(int x, int y);

    }

}