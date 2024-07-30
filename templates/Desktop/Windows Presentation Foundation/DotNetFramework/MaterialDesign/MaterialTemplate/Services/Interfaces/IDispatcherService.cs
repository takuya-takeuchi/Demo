using System;
using System.Threading.Tasks;

namespace MaterialTemplate.Services.Interfaces
{

    public interface IDispatcherService
    {

        Task SafeAction(Action action);

    }

}
