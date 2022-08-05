using Grpc.Core;

namespace Demo.UnitTest.Helpers
{

    internal sealed class TestServerCallContext : ServerCallContext
    {

        #region Fields

        private readonly Metadata _RequestHeaders;

        private readonly CancellationToken _CancellationToken;

        private readonly Metadata _ResponseTrailers;

        private readonly AuthContext _AuthContext;

        private readonly Dictionary<object, object> _UserState;

        private WriteOptions? _WriteOptions;

        #endregion

        #region Constructors

        private TestServerCallContext(Metadata requestHeaders, CancellationToken cancellationToken)
        {
            this._RequestHeaders = requestHeaders;
            this._CancellationToken = cancellationToken;
            this._ResponseTrailers = new Metadata();
            this._AuthContext = new AuthContext(string.Empty, new Dictionary<string, List<AuthProperty>>());
            this._UserState = new Dictionary<object, object>();
        }

        #endregion

        #region Properties

        public Metadata? ResponseHeaders
        {
            get; 
            private set;
        }

        protected override string MethodCore => "MethodName";

        protected override string HostCore => "HostName";

        protected override string PeerCore => "PeerName";

        protected override DateTime DeadlineCore
        {
            get;
        }

        protected override Metadata RequestHeadersCore => this._RequestHeaders;

        protected override CancellationToken CancellationTokenCore => this._CancellationToken;

        protected override Metadata ResponseTrailersCore => this._ResponseTrailers;

        protected override Status StatusCore { get; set; }

        protected override WriteOptions? WriteOptionsCore
        {
            get => this._WriteOptions; 
            set => this._WriteOptions = value;
        }

        protected override AuthContext AuthContextCore => this._AuthContext;

        protected override IDictionary<object, object> UserStateCore => this._UserState;

        #endregion

        #region Methods

        public static TestServerCallContext Create(Metadata? requestHeaders = null, CancellationToken cancellationToken = default)
        {
            return new TestServerCallContext(requestHeaders ?? new Metadata(), cancellationToken);
        }

        #region Overrids

        protected override ContextPropagationToken CreatePropagationTokenCore(ContextPropagationOptions options)
        {
            throw new NotImplementedException();
        }

        protected override Task WriteResponseHeadersAsyncCore(Metadata responseHeaders)
        {
            if (ResponseHeaders != null)
                throw new InvalidOperationException("Response headers have already been written.");

            ResponseHeaders = responseHeaders;
            return Task.CompletedTask;
        }

        #endregion

        #endregion

    }

}
