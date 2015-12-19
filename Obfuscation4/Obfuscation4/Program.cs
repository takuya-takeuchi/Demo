using System;
using System.Reflection;
using Obfuscation4Lib;

namespace Obfuscation4
{
    internal class Program
    {
        private static void Main(string[] args)
        {
            var typeAlice = Assembly.Load("Obfuscation4Lib").GetType("Obfuscation4Lib.Alice");
            if (typeAlice == null)
            {
                Console.WriteLine("Obfuscation4Lib.Alice が見つかりません。");
                return;
            }

            var typeBob = Assembly.Load("Obfuscation4Lib").GetType("Obfuscation4Lib.Bob");
            if (typeBob == null)
            {
                Console.WriteLine("Obfuscation4Lib.Bob が見つかりません。");
                return;
            }

            var alice = (IGreeting)Activator.CreateInstance(typeAlice);
            if (alice == null)
            {
                Console.WriteLine("Obfuscation4Lib.Bob インスタンスの生成に失敗しました。");
                return;
            }

            var bob = (IGreeting)Activator.CreateInstance(typeBob);
            if (bob == null)
            {
                Console.WriteLine("Obfuscation4Lib.Alice インスタンスの生成に失敗しました。");
                return;
            }

            alice.Greeting();
            bob.Greeting();

            var type = typeof (Lovers);
            var method = type.GetMethod("SecretGreeding", BindingFlags.Instance | BindingFlags.NonPublic);
            method.Invoke(alice, new object[] { "Bob" });
            method.Invoke(bob, new object[] { "Alice" });
        }
    }
}