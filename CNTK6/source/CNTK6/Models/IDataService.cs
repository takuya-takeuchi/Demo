using System;

namespace CNTK6.Models
{
    public interface IDataService
    {
        void GetData(Action<DataItem, Exception> callback);
    }
}
