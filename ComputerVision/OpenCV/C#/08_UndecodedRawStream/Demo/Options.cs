using CommandLine;

namespace Demo
{

    internal sealed class Options
    {

        [Option('c', "convert", Required = false, HelpText = "Convert image stream to RGB when capturing")]
        public bool ConvertRgb { get; set; }

        [Option('o', "output", Required = true, HelpText = "Directory path for output captured image")]
        public string Output { get; set; }

    }

}