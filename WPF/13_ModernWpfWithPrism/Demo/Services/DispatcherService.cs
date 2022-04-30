using System;
using System.Threading.Tasks;
using System.Windows.Threading;

using Demo.Services.Interfaces;

namespace Demo.Services
{
    
    public sealed class DispatcherService : IDispatcherService
    {

        #region Fields

        private readonly Dispatcher _Dispatcher;

        #endregion

        #region Constructors
        
        public DispatcherService(Dispatcher dispatcher)
        {
            this._Dispatcher = dispatcher;
        }

        #endregion

        #region IDispatcherService Member

        public async Task SafeAction(Action action)
        {
            if (this._Dispatcher.Thread.IsAlive)
            {
                try
                {
                    if (this._Dispatcher.CheckAccess())
                    {
                        action.Invoke();
                    }
                    else
                    {
                        await Dispatcher.CurrentDispatcher.BeginInvoke(action);
                    }
                }
                catch
                {
                }
            }
        }

        #endregion

    }

}
