using System;
using System.Linq;
using System.Security.Cryptography;
using System.Text;

using Android.Security.Keystore;
using Java.Security;
using Javax.Crypto;

using Demo.Services.Interfaces;

using CipherMode = Javax.Crypto.CipherMode;

namespace Demo.Droid.Services
{

    public sealed class DeviceService : IDeviceService
    {

        #region Fields

        private readonly ILoggingService _LoggingService;

        private static readonly string KeyAlgorithm = KeyProperties.KeyAlgorithmRsa;

        private static readonly int KeySizeInBits = 2048;

        private static readonly string Padding = KeyProperties.EncryptionPaddingRsaPkcs1;

        private static readonly string CipherTransformation = "RSA/ECB/PKCS1Padding";

        private static readonly string AndroidKeyStore = "AndroidKeyStore";

        private static readonly string KeyTag = "apps1";


        #endregion

        #region Constructors

        public DeviceService(ILoggingService loggingService)
        {
            this._LoggingService = loggingService;
        }

        #endregion

        #region Methods

        #region Helpers

        private KeyPair GenerateAndStoreKeyPairInKeyChain(string alias)
        {
            try
            {
                var keyStore = KeyStore.GetInstance(AndroidKeyStore);
                if (keyStore == null)
                    return null;

                keyStore.Load(null);

                var keyPairGenerator = KeyPairGenerator.GetInstance(KeyAlgorithm, AndroidKeyStore);
                if (keyPairGenerator == null)
                    return null;

                keyPairGenerator.Initialize(new KeyGenParameterSpec.Builder(alias,
                                                                            KeyStorePurpose.Encrypt | KeyStorePurpose.Decrypt)
                                .SetEncryptionPaddings(Padding)
                                .SetKeySize(KeySizeInBits)
                                .Build());

                var keyPair = keyPairGenerator.GenerateKeyPair();
                return new KeyPair(keyPair.Public, keyPair.Private);
            }
            catch (Exception e)
            {
                this._LoggingService.Error(e, null, "Failed to generate key pair");
                return null;
            }
        }
        
        private KeyPair RetrieveKeyPairFromKeyChain(string alias)
        {
            try
            {
                var keyStore = KeyStore.GetInstance(AndroidKeyStore);
                if (keyStore == null)
                    return null;

                keyStore.Load(null);

                var privateKeyEntry = (KeyStore.PrivateKeyEntry)keyStore.GetEntry(alias, null);
                if (privateKeyEntry?.PrivateKey == null)
                    return null;

                var publicKey = keyStore.GetCertificate(alias)?.PublicKey;
                if (publicKey == null)
                    return null;

                var privateKey = privateKeyEntry.PrivateKey;
                return new KeyPair(publicKey, privateKey);
            }
            catch (Exception e)
            {
                this._LoggingService.Error(e, null, "Failed to get key pair from key chain");
                return null;
            }
        }

        #endregion

        #endregion

        #region IDeviceService Members

        public string GetKey()
        {
            KeyPair keyPair = null;

            try
            {
                this._LoggingService.Info("Find KeyPair");
                keyPair = this.RetrieveKeyPairFromKeyChain(KeyTag);
                if (keyPair == null)
                {
                    this._LoggingService.Info("Generate and Store KeyPair");
                    keyPair = this.GenerateAndStoreKeyPairInKeyChain(KeyTag);
                }

                var plainText = Encoding.ASCII.GetBytes(KeyTag);
                var cipherEncryption = Cipher.GetInstance(CipherTransformation);
                cipherEncryption.Init(CipherMode.EncryptMode, keyPair.PublicKey);
                var cipherText = cipherEncryption.DoFinal(plainText);

                // check private key
                var cipherDecryption = Cipher.GetInstance(CipherTransformation);
                cipherDecryption.Init(CipherMode.DecryptMode, keyPair.PrivateKey);
                plainText = cipherDecryption.DoFinal(cipherText);
                
                var text = Encoding.ASCII.GetString(plainText);
                if (!KeyTag.Equals(text))
                    this._LoggingService.Error("Failed to decrypt from encrypted text");

                // Android does not allow to export private key
                //var binaryPrivate = keyPair.PrivateKey.GetEncoded();
                var binaryPublic = keyPair.PublicKey.GetEncoded();
                using (var sha1 = SHA1.Create())
                {
                    //var hash = string.Join("", sha1.ComputeHash(binaryPrivate.Concat(binaryPublic).ToArray()).Select(b => $"{b:X}"));
                    var hash = string.Join("", sha1.ComputeHash(binaryPublic).Select(b => $"{b:X}"));
                    return hash;
                }
            }
            catch (Exception exception)
            {
                this._LoggingService.Error(exception, null, "Failed to get key");
                return "-";
            }
            finally
            {
                keyPair?.PrivateKey?.Dispose();
                keyPair?.PublicKey?.Dispose();
            }
        }

        public void Reset()
        {
            try
            {
                var keyStore = KeyStore.GetInstance(AndroidKeyStore);
                if (keyStore == null)
                    return;

                keyStore.Load(null);

                keyStore.DeleteEntry(KeyTag);
            }
            catch (Exception e)
            {
                this._LoggingService.Error(e, null, "Failed to delete key");
            }
        }

        #endregion

        private sealed class KeyPair
        {

            #region Constructors

            public KeyPair(IPublicKey publicKey, IPrivateKey privateKey)
            {
                this.PublicKey = publicKey;
                this.PrivateKey = privateKey;
            }

            #endregion

            #region Properties

            public IPrivateKey PrivateKey
            {
                get;
            }

            public IPublicKey PublicKey
            {
                get;
            }

            #endregion

        }

    }

}