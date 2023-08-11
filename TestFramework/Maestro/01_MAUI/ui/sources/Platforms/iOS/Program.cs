using System.IO;
using System.Runtime.InteropServices;

using ObjCRuntime;
using UIKit;

namespace Demo;

public class Program
{
	// This is the main entry point of the application.
	static void Main(string[] args)
	{
		UIApplication.Main(args, null, typeof(AppDelegate));
	}
}
