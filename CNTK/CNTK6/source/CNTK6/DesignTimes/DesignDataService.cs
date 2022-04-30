using System;
using CNTK6.Models;

namespace CNTK6.DesignTimes
{
    public class DesignDataService : IDataService
    {
        public void GetData(Action<DataItem, Exception> callback)
        {
            // Use this to create design time data
        }
    }
}