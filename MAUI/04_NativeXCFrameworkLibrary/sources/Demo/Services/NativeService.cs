using Demo.Services.Interfaces;

namespace Demo.Services
{

    public sealed class NativeService : INativeService
    {

        public int Add(int x, int y)
        {
            return NativeSharp.Native.Add(x, y);
        }

    }

}
