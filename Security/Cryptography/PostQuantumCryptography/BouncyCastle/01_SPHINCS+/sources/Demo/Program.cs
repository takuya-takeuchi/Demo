using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using NLog;
using Org.BouncyCastle.Pqc.Crypto.SphincsPlus;
using Org.BouncyCastle.Security;

namespace Demo
{

    internal sealed class Program
    {

        #region Fields

        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        #endregion

        #region Methods

        private static void Main(string[] args)
        {
            if (args.Length != 2)
            {
                Logger.Error($"{nameof(Demo)} <plain-text> <method>");
                return;
            }

            var message = args[0];
            var method = args[1];

            var random = new SecureRandom();
            var keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.haraka_128f);

            if (method == "haraka_128f")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.haraka_128f);
            if (method == "haraka_128f_simple")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.haraka_128f_simple);
            if (method == "haraka_128s")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.haraka_128s);
            if (method == "haraka_128s_simple")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.haraka_128s_simple);
            if (method == "haraka_192f")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.haraka_192f);
            if (method == "haraka_192f_simple")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.haraka_192f_simple);
            if (method == "haraka_192s")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.haraka_192s);
            if (method == "haraka_192s_simple")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.haraka_192s_simple);
            if (method == "haraka_256f")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.haraka_256f);
            if (method == "haraka_256f_simple")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.haraka_256f_simple);
            if (method == "haraka_256s")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.haraka_256s);
            if (method == "haraka_256s_simple")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.haraka_256s_simple);
            if (method == "sha2_128f")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.sha2_128f);
            // if (method == "sha2_128f_robust")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.sha2_128f_robust);
            if (method == "sha2_128s")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.sha2_128s);
            // if (method == "sha2_128s_robust")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.sha2_128s_robust);
            if (method == "sha2_192f")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.sha2_192f);
            // if (method == "sha2_192f_robust")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.sha2_192f_robust);
            if (method == "sha2_192s")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.sha2_192s);
            // if (method == "sha2_192s_robust")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.sha2_192s_robust);
            if (method == "sha2_256f")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.sha2_256f);
            // if (method == "sha2_256f_robust")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.sha2_256f_robust);
            if (method == "sha2_256s")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.sha2_256s);
            // if (method == "sha2_256s_robust")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.sha2_256s_robust);
            if (method == "shake_128f")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.shake_128f);
            // if (method == "shake_128f_robust")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.shake_128f_robust);
            if (method == "shake_128s")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.shake_128s);
            // if (method == "shake_128s_robust")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.shake_128s_robust);
            if (method == "shake_192f")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.shake_192f);
            // if (method == "shake_192f_robust")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.shake_192f_robust);
            if (method == "shake_192s")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.shake_192s);
            // if (method == "shake_192s_robust")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.shake_192s_robust);
            if (method == "shake_256f")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.shake_256f);
            // if (method == "shake_256f_robust")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.shake_256f_robust);
            if (method == "shake_256s")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.shake_256s);
            // if (method == "shake_256s_robust")  keyGenParameters = new SphincsPlusKeyGenerationParameters(random, SphincsPlusParameters.shake_256s_robust);
            
            var keyPairGen = new SphincsPlusKeyPairGenerator();
            keyPairGen.Init(keyGenParameters);
            var keyPair = keyPairGen.GenerateKeyPair();

            var pubKey = (SphincsPlusPublicKeyParameters)keyPair.Public;
            var privKey = (SphincsPlusPrivateKeyParameters)keyPair.Private;

            // Signing 
            var aliceSign = new SphincsPlusSigner();
            aliceSign.Init(true, privKey);
            var signature = aliceSign.GenerateSignature(System.Text.Encoding.UTF8.GetBytes(message));

            // verify signature
            var bobVerify = new SphincsPlusSigner();
            bobVerify.Init(false, pubKey);
            var rtn = bobVerify.VerifySignature(System.Text.Encoding.UTF8.GetBytes(message), signature);

            Logger.Info("                   Message: {0}", message);
            Logger.Info("                    Method: {0}", method);

            Logger.Info("       Public key (length): {0} bytes", pubKey.GetEncoded().Length);
            Logger.Info("         Alice Public key : {0}", Convert.ToHexString(pubKey.GetEncoded()));
            Logger.Info("      Private key (length): {0} bytes", privKey.GetEncoded().Length);
            Logger.Info("         Alice Private key: {0}", Convert.ToHexString(privKey.GetEncoded()));

            Logger.Info("        Signature (length): {0} bytes", signature.Length);
            Logger.Info("Signature (first 50 bytes): {0}", Convert.ToHexString(signature)[..100]);
            Logger.Info("                  Verified: {0}", rtn);
        }

        #endregion

    }

}
