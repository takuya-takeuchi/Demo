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
		var huhn = System.Runtime.InteropServices.NativeLibrary.Load(Path.Combine("libLuhn.dylib"));
		var huhnc = System.Runtime.InteropServices.NativeLibrary.Load(Path.Combine("libLuhnc.dylib"));

		// if you want to use a different Application Delegate class from "AppDelegate"
		// you can specify it here.
		UIApplication.Main(args, null, typeof(AppDelegate));
	}
}
