using Demo.Services.Interfaces;

namespace Demo.Droid.Services
{

    public sealed class NativeService : INativeService
    {

        #region INativeService Members

        public string Message()
        {
            var hello = new Com.Example.Hello.Hello();
            return hello.Message();
        }

        #endregion

    }

}