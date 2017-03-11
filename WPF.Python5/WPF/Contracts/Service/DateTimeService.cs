using System;
using System.ServiceModel;
using System.ServiceModel.Web;

namespace WPFPython.Contracts.Service
{

    [ServiceBehavior(InstanceContextMode = InstanceContextMode.Single)]
    public sealed class DateTimeService : IDateTimeService
    {

        #region イベント

        public event EventHandler<DateTime> DateTimeReceived;

        #endregion

        #region メソッド

        [WebInvoke(Method = "POST", UriTemplate = "Send?dateTime={paramDateTime}",
          RequestFormat = WebMessageFormat.Json)]
        public void Send(string paramDateTime)
        {
            var dt = DateTime.Parse(paramDateTime);
            this.DateTimeReceived?.Invoke(this, dt);
        }

        #endregion

    }

}