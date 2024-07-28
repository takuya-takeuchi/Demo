using System.Collections.Generic;
using MaterialTemplate.Services.Interfaces;
using NLog;

namespace MaterialTemplate.Services
{

    public sealed class NLogLogFactoryService : ILogFactoryService
    {

        #region ILogFactoryService Members

        public ILogService Create()
        {
            return new NLogLogService(LogManager.GetCurrentClassLogger());
        }

        public ILogService Create(string name)
        {
            return new NLogLogService(LogManager.GetLogger(name));
        }

        public ILogService Create(string name, IDictionary<string, string> variables)
        {
            return new NLogLogService(name, variables);
        }

        #endregion

    }

}
