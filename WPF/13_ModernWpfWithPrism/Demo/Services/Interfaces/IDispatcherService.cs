using System;
using System.Threading.Tasks;

namespace Demo.Services.Interfaces
{

    public interface IDispatcherService
    {

        Task SafeAction(Action action);

    }

}