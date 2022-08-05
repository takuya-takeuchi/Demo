namespace Demo.Test;

public abstract class TestBase<T>
    where T: Grpc.Core.ClientBase
{

    #region Properties

    protected T Client
    {
        get;
        set;
    }

    #endregion

}