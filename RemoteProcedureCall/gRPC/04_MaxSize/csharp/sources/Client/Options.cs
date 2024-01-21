using CommandLine;

namespace Demo;

public class Options
{

    [Option('r', "receive", Required = false, Default = null, HelpText = "max receive message length")]
    public int? MaxReceiveMessageLength { get; set; }

    [Option('s', "send", Required = false, Default = null, HelpText = "max send message length")]
    public int? MaxSendMMessageLength { get; set; }

    [Option('q', "request data size", Required = true, Default = null, HelpText = "data size of request")]
    public uint RequestDataSize { get; set; }

    [Option('p', "response data size", Required = true, Default = null, HelpText = "data size of response")]
    public uint ResponseDataSize { get; set; }

}