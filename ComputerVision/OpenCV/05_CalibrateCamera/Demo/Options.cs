using CommandLine;

namespace Demo
{

    internal sealed class Options
    {

        [Option('h', "horizontal", Required = true, HelpText = "Number of inner corners (horizontal)")]
        public int HorizontalPatternSize { get; set; }

        [Option('v', "vertical", Required = true, HelpText = "Number of inner corners (vertical)")]
        public int VerticalPatternSize { get; set; }

        [Option('s', "size", Required = true, HelpText = "Square size (cm)")]
        public float Size { get; set; }

        [Option('c', "count", Required = true, HelpText = "Reference count")]
        public int Count { get; set; }

        [Option('o', "output", Required = false, HelpText = "Output captured image")]
        public bool Output { get; set; }

    }

}