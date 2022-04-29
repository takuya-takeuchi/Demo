using CommandLine;

namespace Demo
{

    internal sealed class Options
    {

        [Option('s', "size", Required = true, HelpText = "Square size (m)")]
        public float Size { get; set; }

        [Option('d', "dictionary", Required = true, HelpText = "ArUco predefine dictinary name")]
        public string Dictionary { get; set; }

        [Option('p', "parameter", Required = true, HelpText = "Path of Camera intrinsic parameter")]
        public string Parameter { get; set; }

        [Option('o', "output", Required = false, HelpText = "Output captured image")]
        public bool Output { get; set; }

    }

}