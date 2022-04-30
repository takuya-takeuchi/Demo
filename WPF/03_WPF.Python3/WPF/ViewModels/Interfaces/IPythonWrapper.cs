using System.Collections.Generic;

namespace WPFPython.ViewModels.Interfaces
{

    public interface IPythonWrapper
    {

        Dictionary<string, object> load(string path);

    }

}
