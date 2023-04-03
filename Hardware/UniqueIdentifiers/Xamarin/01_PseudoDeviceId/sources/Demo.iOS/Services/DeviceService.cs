using System;
using System.Linq;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text;

using Foundation;
using Security;

using Demo.Services.Interfaces;
using System.Security.Policy;

namespace Demo.iOS.Services
{

    public sealed class DeviceService : IDeviceService
    {

        #region Fields

        private readonly ILoggingService _LoggingService;

        private static readonly SecKeyType SecKeyType = SecKeyType.RSA;

        private static readonly int KeySizeInBits = 2048;

        private static readonly SecPadding SecPadding = SecPadding.PKCS1;

        private static readonly SecAccessible SecAccessible = SecAccessible.Always;

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

        private static KeyPair GenerateAndStoreKeyPairInKeyChain(string keyTag)
        {
            var keyPairGenerationParameters = new SecKeyGenerationParameters
            {
                KeyType = SecKeyType,
                KeySizeInBits = KeySizeInBits,
                CanEncrypt = true,
                CanDecrypt = true
            };

            var privateKey = SecKey.CreateRandomKey(keyPairGenerationParameters, out var error);
            var publicKey = privateKey?.GetPublicKey();

            using (var privateKeyRecord = new SecRecord(SecKind.Key)
            {
                ApplicationTag = keyTag,
                Accessible = SecAccessible,
                ValueData = privateKey?.GetExternalRepresentation(),
                KeyClass = SecKeyClass.Private,
                KeySizeInBits = KeySizeInBits,
                KeyType = SecKeyType
            })
            using (var publicKeyRecord = new SecRecord(SecKind.Key)
            {
                ApplicationTag = keyTag,
                Accessible = SecAccessible,
                ValueData = publicKey?.GetExternalRepresentation(),
                KeyClass = SecKeyClass.Public,
                KeySizeInBits = KeySizeInBits,
                KeyType = SecKeyType
            })
            {
                // Add keys to key chain
                // Application must have permission to access key chain
                do
                {
                    var statusCode = SecKeyChain.Add(privateKeyRecord);
                    if (statusCode == SecStatusCode.DuplicateItem)
                    {
                        SecKeyChain.Remove(privateKeyRecord);
                        continue;
                    }

                    break;
                } while (true);
                do
                {
                    var statusCode = SecKeyChain.Add(publicKeyRecord);
                    if (statusCode == SecStatusCode.DuplicateItem)
                    {
                        SecKeyChain.Remove(publicKeyRecord);
                        continue;
                    }

                    break;
                } while (true);
            }

            return new KeyPair(publicKey, privateKey);
        }
        
        private KeyPair RetrieveKeyPairFromKeyChain(string keyTag)
        {
            try
            {
                using (var privateQuery = new SecRecord(SecKind.Key)
                {
                    ApplicationTag = NSData.FromString(keyTag),
                    KeyClass = SecKeyClass.Private
                })
                using (var publicQuery = new SecRecord(SecKind.Key)
                {
                    ApplicationTag = NSData.FromString(keyTag),
                    KeyClass = SecKeyClass.Public
                })
                using (var s = SecKeyChain.QueryAsData(privateQuery, false, out var error1))
                using (var p = SecKeyChain.QueryAsData(publicQuery, false, out var error2))
                {
                    if (error1 != SecStatusCode.Success)
                        return null;
                    if (error2 != SecStatusCode.Success)
                        return null;

                    var keyPairGenerationParameters = new SecKeyGenerationParameters
                    {
                        KeyType = SecKeyType,
                        KeySizeInBits = KeySizeInBits,
                        CanEncrypt = true,
                        CanDecrypt = true
                    };

                    var privateKey = SecKey.Create(s, SecKeyType, SecKeyClass.Private, KeySizeInBits, keyPairGenerationParameters.Dictionary, out var error3);
                    var publicKey = SecKey.Create(p, SecKeyType, SecKeyClass.Public, KeySizeInBits, keyPairGenerationParameters.Dictionary, out var error4);
                    if (error3 != null)
                        return null;
                    if (error4 != null)
                        return null;
                    return new KeyPair(publicKey, privateKey);
                }
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
                keyPair = RetrieveKeyPairFromKeyChain(KeyTag);
                if (keyPair == null)
                {
                    this._LoggingService.Info("Generate and Store KeyPair");
                    keyPair = GenerateAndStoreKeyPairInKeyChain(KeyTag);
                }

                var plainText = Encoding.ASCII.GetBytes(KeyTag);
                var statusCode = keyPair.PublicKey.Encrypt(SecPadding, plainText, out var cipherText);
                
                // check private key
                statusCode = keyPair.PrivateKey.Decrypt(SecPadding, cipherText, out plainText);
                var text = Encoding.ASCII.GetString(plainText);
                if (!KeyTag.Equals(text))
                    this._LoggingService.Error($"Failed to decrypt from encrypted text [{statusCode}]");

                using (var @private = keyPair.PrivateKey.GetExternalRepresentation())
                using (var @public = keyPair.PublicKey.GetExternalRepresentation())
                {
                    var binaryPrivate = new byte[@private.Length];
                    var binaryPublic = new byte[@public.Length];
                    Marshal.Copy(@private.Bytes, binaryPrivate, 0, binaryPrivate.Length);
                    Marshal.Copy(@public.Bytes, binaryPublic, 0, binaryPublic.Length);
                    using (var sha1 = SHA1.Create())
                    {
                        var hash = string.Join("", sha1.ComputeHash(binaryPrivate.Concat(binaryPublic).ToArray()).Select(b => $"{b:X}"));
                        return hash;
                    }
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
            using (var privateKeyRecord = new SecRecord(SecKind.Key)
            {
                ApplicationTag = KeyTag,
                Accessible = SecAccessible,
                KeyClass = SecKeyClass.Private,
                KeySizeInBits = KeySizeInBits,
                KeyType = SecKeyType
            })
            using (var publicKeyRecord = new SecRecord(SecKind.Key)
            {
                ApplicationTag = KeyTag,
                Accessible = SecAccessible,
                KeyClass = SecKeyClass.Public,
                KeySizeInBits = KeySizeInBits,
                KeyType = SecKeyType
            })
            {
                var statusCode = SecKeyChain.Remove(privateKeyRecord);
                this._LoggingService.Info($"Remove private key [{statusCode}]");
                statusCode = SecKeyChain.Remove(publicKeyRecord);
                this._LoggingService.Info($"Remove public key [{statusCode}]");
            }
        }

        #endregion

        private sealed class KeyPair
        {

            #region Constructors

            public KeyPair(SecKey publicKey, SecKey privateKey)
            {
                this.PublicKey = publicKey;
                this.PrivateKey = privateKey;
            }

            #endregion

            #region Properties

            public SecKey PrivateKey
            {
                get;
            }

            public SecKey PublicKey
            {
                get;
            }

            #endregion

        }

    }

}