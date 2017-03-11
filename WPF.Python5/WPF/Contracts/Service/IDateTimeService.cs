using System;
using System.ServiceModel;

namespace WPFPython.Contracts.Service
{
    
    [ServiceContract]
    public interface IDateTimeService
    {

        event EventHandler<DateTime> DateTimeReceived; 

        [OperationContract]
        void Send(string paramDateTime);

    }

}
