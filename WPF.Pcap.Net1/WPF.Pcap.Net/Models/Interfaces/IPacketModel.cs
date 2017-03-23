using System;

namespace WPF.PcapNet.Models.Interfaces
{

    public interface IPacketModel
    {

        #region Properties

        string Destination
        {
            get;
        }

        int Length
        {
            get;
        }

        DateTime Timestamp
        {
            get;
        }

        string Source
        {
            get;
        }

        #endregion

    }

}