using System;

using Demo.Controls;
using Demo.Services.Interfaces;

namespace Demo.Services
{

    public sealed class DeviceOrientationDetectService : IDeviceOrientationDetectService
    {

        #region IDeviceOrientationDetectService Members
        
        public event EventHandler<Orientation> OrientationChanged;

        public void NotifyOrientationChange(Orientation orientation)
        {
            this.OrientationChanged?.Invoke(this, orientation);
        }

        #endregion

    }

}
