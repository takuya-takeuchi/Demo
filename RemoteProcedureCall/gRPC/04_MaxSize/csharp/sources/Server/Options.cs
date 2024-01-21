using CommandLine;

namespace Demo;

public class Options
{

    [Option('r', "receive", Required = false, Default = null, HelpText = "max receive message length")]
    public int? MaxReceiveMessageLength { get; set; }

    [Option('s', "send", Required = false, Default = null, HelpText = "max send message length")]
    public int? MaxSendMessageLength { get; set; }

}