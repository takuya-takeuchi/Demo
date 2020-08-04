using Xamarin.Native.Nuget.Services.Interfaces;

namespace Xamarin.Native.Nuget.iOS.Services
{

    public sealed class NativeService : INativeService
    {

        public int Add(int x, int y)
        {
            return NativeSharp.Native.Add(x, y);
        }

    }

}
