using System;

using Demo.Controls;

namespace Demo.Services.Interfaces
{

    public interface IDeviceOrientationDetectService
    {

        event EventHandler<Orientation> OrientationChanged;

        void NotifyOrientationChange(Orientation orientation);

    }

}
