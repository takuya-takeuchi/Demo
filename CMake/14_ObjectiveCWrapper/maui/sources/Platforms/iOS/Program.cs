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
		// Load librariesLuhn.xcframework/ios-arm64/libLuhn.dylib
		System.Runtime.InteropServices.NativeLibrary.Load(Path.Combine("Frameworks", "Luhn.xcframework", "ios-arm64", "libLuhn.dylib"));
		System.Runtime.InteropServices.NativeLibrary.Load(Path.Combine("Frameworks", "Luhnc.xcframework", "ios-arm64", "libLuhnc.dylib"));

		// if you want to use a different Application Delegate class from "AppDelegate"
		// you can specify it here.
		UIApplication.Main(args, null, typeof(AppDelegate));
	}
}
