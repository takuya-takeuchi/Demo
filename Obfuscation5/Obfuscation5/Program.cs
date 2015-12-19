using System.Reflection;
using Obfuscation5Lib;

namespace Obfuscation5
{
    internal class Program
    {
        private static void Main(string[] args)
        {
            var alice = new Alice();
            var bob = new Bob();

            alice.Greeting();
            bob.Greeting();

            var type = typeof (Lovers);
            var method = type.GetMethod("SecretGreeding", BindingFlags.Instance | BindingFlags.NonPublic);
            method.Invoke(alice, new object[] { "Bob" });
            method.Invoke(bob, new object[] { "Alice" });
        }
    }
}