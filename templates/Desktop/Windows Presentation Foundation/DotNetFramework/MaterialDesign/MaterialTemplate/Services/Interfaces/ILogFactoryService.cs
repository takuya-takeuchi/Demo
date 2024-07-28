using System.Collections.Generic;

namespace MaterialTemplate.Services.Interfaces
{

    public interface ILogFactoryService
    {

        ILogService Create();

        ILogService Create(string name);

        ILogService Create(string name, IDictionary<string, string> variables);

    }

}
