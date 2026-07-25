using System.Threading;

namespace Demo.Services
{

    public sealed class RefreshMetrics
    {

        #region Fields

        private int _Active;

        private int _Peak;

        #endregion

        #region Properties

        public int Active => Volatile.Read(ref _Active);

        public int Peak => Volatile.Read(ref _Peak);

        #endregion

        #region Methods

        public int Enter()
        {
            int active = Interlocked.Increment(ref _Active);

            while (true)
            {
                int currentPeak = Volatile.Read(ref _Peak);
                if (active <= currentPeak)
                    break;

                if (Interlocked.CompareExchange(ref _Peak, active, currentPeak) == currentPeak)
                    break;
            }

            return active;
        }

        public int Exit()
        {
            return Interlocked.Decrement(ref _Active);
        }

        public void Reset()
        {
            Volatile.Write(ref _Peak, Active);
        }

        #endregion

    }

}