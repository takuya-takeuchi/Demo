using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.IdentityModel.Tokens;
using System.Net;
using System.Security.Cryptography;
using System.ServiceModel;
using System.ServiceModel.Channels;
using System.ServiceModel.Description;
using System.ServiceModel.Dispatcher;
using System.Text;
using System.Threading.Tasks;
using System.Xml;
using System.Xml.Linq;
using Onvif.DeviceManagement;
using Onvif.Media;
using DateTime = System.DateTime;

namespace Onvif1
{


    internal class Program
    {

        private static async Task Main(string[] args)
        {
            if (args.Length != 3)
            {
                Console.WriteLine($"[Error] {nameof(Onvif1)} <url> <user> <password>");
                return;
            }

            var url = args[0];
            var user = args[1];
            var password = args[2];
            Console.WriteLine($"[Info]      Url: {url}");
            Console.WriteLine($"[Info]     User: {user}");
            Console.WriteLine($"[Info] Password: {password}");

            var httpBinding = new HttpTransportBindingElement
            {
                AuthenticationScheme = AuthenticationSchemes.Digest
            };
            var messageElement = new TextMessageEncodingBindingElement
            {
                MessageVersion = MessageVersion.CreateVersion(EnvelopeVersion.Soap12, AddressingVersion.None)
            };
            var binding = new CustomBinding(messageElement, httpBinding);
            var address = new EndpointAddress(url);
            
            var deviceClient = new DeviceClient(binding, address);
            deviceClient.ClientCredentials.UserName.UserName = user;
            deviceClient.ClientCredentials.UserName.Password = password;

            Console.WriteLine();
            Console.WriteLine($"[Info] Invoke {nameof(deviceClient.GetSystemDateAndTimeAsync)}");
            var dt = await deviceClient.GetSystemDateAndTimeAsync();
            var deviceTime = new DateTime(dt.UTCDateTime.Date.Year, dt.UTCDateTime.Date.Month, dt.UTCDateTime.Date.Day, dt.UTCDateTime.Time.Hour, dt.UTCDateTime.Time.Minute, dt.UTCDateTime.Time.Second);
            var deviceTimeOffset = (deviceTime - DateTime.UtcNow).TotalSeconds;
            Console.WriteLine($"[Info] \t{deviceTime:yyyy/M/dd hh:mm:ss.fff}");
            
            Console.WriteLine();
            Console.WriteLine($"[Info] Invoke {nameof(deviceClient.GetCapabilitiesAsync)}");
            var serviceCapabilities = await deviceClient.GetCapabilitiesAsync(new[]
            {
                CapabilityCategory.All
            });

            var media = serviceCapabilities.Capabilities.Media;
            Console.WriteLine($"[Info] \t{nameof(serviceCapabilities.Capabilities.Media)}.XAddr: {media.XAddr}");
            Console.WriteLine($"[Info] \t\t{nameof(media.StreamingCapabilities),25}");
            Console.WriteLine($"[Info] \t\t{nameof(media.StreamingCapabilities.RTPMulticast),25}: {media.StreamingCapabilities.RTPMulticast}");
            Console.WriteLine($"[Info] \t\t{nameof(media.StreamingCapabilities.RTP_RTSP_TCP),25}: {media.StreamingCapabilities.RTP_RTSP_TCP}");
            Console.WriteLine($"[Info] \t\t{nameof(media.StreamingCapabilities.RTP_TCP),25}: {media.StreamingCapabilities.RTP_TCP}");
            Console.WriteLine($"[Info] \t\t{nameof(media.StreamingCapabilities.RTPMulticastSpecified),25}: {media.StreamingCapabilities.RTPMulticastSpecified}");
            Console.WriteLine($"[Info] \t\t{nameof(media.StreamingCapabilities.RTP_RTSP_TCPSpecified),25}: {media.StreamingCapabilities.RTP_RTSP_TCPSpecified}");
            Console.WriteLine($"[Info] \t\t{nameof(media.StreamingCapabilities.RTP_TCPSpecified),25}: {media.StreamingCapabilities.RTP_TCPSpecified}");
            
            var mediaAddress = new EndpointAddress(media.XAddr);
            var mediaClient = new MediaClient(binding, mediaAddress);
            mediaClient.ClientCredentials.UserName.UserName = user;
            mediaClient.ClientCredentials.UserName.Password = password;
            var behavior = new PasswordDigestBehavior(user, password, deviceTimeOffset);
            mediaClient.Endpoint.EndpointBehaviors.Add(behavior);

            Console.WriteLine();
            Console.WriteLine($"[Info] Invoke {nameof(mediaClient.GetProfilesAsync)}");
            var profiles = await mediaClient.GetProfilesAsync();
            foreach (var profile in profiles.Profiles)
            {
                Console.WriteLine($"[Info] \t{nameof(profile.Name),10}: {profile.Name}");
                Console.WriteLine($"[Info] \t\t{nameof(profile.token),10}: {profile.token}");
            }

            Console.WriteLine();
            Console.WriteLine($"[Info] Invoke {nameof(mediaClient.GetSnapshotUriAsync)}");
            var snapshotUri = await mediaClient.GetSnapshotUriAsync(profiles.Profiles[0].token);
            Console.WriteLine($"[Info] \t{nameof(snapshotUri.Uri),10}: {snapshotUri.Uri}");

            Console.WriteLine();
            Console.WriteLine($"[Info] Invoke {nameof(mediaClient.GetStreamUriAsync)}");
            var streamUri = await mediaClient.GetStreamUriAsync(new StreamSetup()
            {
                Stream = StreamType.RTPUnicast,
                Transport = new Transport
                {
                    Protocol = TransportProtocol.HTTP,
                },
            },  profiles.Profiles[0].token);
            Console.WriteLine($"[Info] \t{nameof(streamUri.Uri),10}: {streamUri.Uri}");
        }


        public class UsernameToken : SecurityToken
        {
            UsernameInfo _usernameInfo;
            ReadOnlyCollection<SecurityKey> _securityKeys;
            DateTime _created = DateTime.UtcNow; // DateTime.Now;
            DateTime _expiration = DateTime.Now + new TimeSpan(10, 0, 0);
            Guid _id = Guid.NewGuid();
            byte[] _nonce = new byte[16];

            public UsernameToken(UsernameInfo usernameInfo, string nonce, string created)
            {
                if (usernameInfo == null)
                    throw new ArgumentNullException("usernameInfo");

                _usernameInfo = usernameInfo;

                if (nonce != null)
                {
                    _nonce = Convert.FromBase64String(nonce);
                }

                if (created != null)
                {
                    _created = DateTime.Parse(created);
                }

                // the user name token is not capable of any crypto
                _securityKeys = new ReadOnlyCollection<SecurityKey>(new List<SecurityKey>());
            }

            public UsernameToken(UsernameInfo usernameInfo) : this(usernameInfo, null, null) { }

            /// <summary>
            /// UsernameToken that allows for time offset corrections in case device time and client time
            /// aren't synchronized.  
            /// </summary>
            /// <param name="usernameInfo"></param>
            /// <param name="DeviceTimeOffset">Difference between client UTC time and device UTC time in seconds</param>
            public UsernameToken(UsernameInfo usernameInfo, double DeviceTimeOffset) : this(usernameInfo, null, null)
            {
                _created = DateTime.UtcNow + TimeSpan.FromSeconds(DeviceTimeOffset);
            }

            public UsernameInfo UsernameInfo { get { return _usernameInfo; } }

            public override ReadOnlyCollection<SecurityKey> SecurityKeys { get { return _securityKeys; } }

            public override DateTime ValidFrom { get { return _created; } }
            public override DateTime ValidTo { get { return _expiration; } }
            public override string Id { get { return _id.ToString(); } }

            public string GetPasswordDigestAsBase64()
            {
                // generate a cryptographically strong random value
                RandomNumberGenerator rndGenerator = new RNGCryptoServiceProvider();
                rndGenerator.GetBytes(_nonce);

                // get other operands to the right format
                byte[] time = Encoding.UTF8.GetBytes(GetCreatedAsString());
                byte[] pwd = Encoding.UTF8.GetBytes(_usernameInfo.Password);
                byte[] operand = new byte[_nonce.Length + time.Length + pwd.Length];
                Array.Copy(_nonce, operand, _nonce.Length);
                Array.Copy(time, 0, operand, _nonce.Length, time.Length);
                Array.Copy(pwd, 0, operand, _nonce.Length + time.Length, pwd.Length);

                // create the hash
                SHA1 sha1 = SHA1.Create();
                return Convert.ToBase64String(sha1.ComputeHash(operand));
            }

            public string GetNonceAsBase64()
            {
                return Convert.ToBase64String(_nonce);
            }

            public string GetCreatedAsString()
            {
                return XmlConvert.ToString(_created.ToUniversalTime(), "yyyy-MM-ddTHH:mm:ssZ");
            }

            public bool ValidateToken(string password)
            {
                byte[] pwd = Encoding.UTF8.GetBytes(password);
                byte[] createdBytes = Encoding.UTF8.GetBytes(GetCreatedAsString());
                byte[] operand = new byte[_nonce.Length + createdBytes.Length + pwd.Length];
                Array.Copy(_nonce, operand, _nonce.Length);
                Array.Copy(createdBytes, 0, operand, _nonce.Length, createdBytes.Length);
                Array.Copy(pwd, 0, operand, _nonce.Length + createdBytes.Length, pwd.Length);
                SHA1 sha1 = SHA1.Create();
                string trueDigest = Convert.ToBase64String(sha1.ComputeHash(operand));

                return String.Compare(trueDigest, _usernameInfo.Password) == 0;
            }

            public XmlElement GetXml(XmlDocument xml)
            {
                xml = new XmlDocument();

                XNamespace wsu = "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd";
                XNamespace wsse = "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd";

                XDocument xDoc = new XDocument(
                    new XElement(wsse + "UsernameToken",
                        // wsu:Id Security token
                        new XElement(wsse + "Username", _usernameInfo.Username),
                        new XElement(wsse + "Password", GetPasswordDigestAsBase64(),
                            new XAttribute("Type", "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordDigest")),
                        new XElement(wsse + "Nonce", GetNonceAsBase64()),
                        new XElement(wsu + "Created", GetCreatedAsString()),
                        new XAttribute(XNamespace.Xmlns + "wsu", wsu),
                        new XAttribute(XNamespace.Xmlns + "wsse", wsse)
                        )
                    );

                var doc = new XmlDocument();
                doc.LoadXml(xDoc.ToString());
                return doc.DocumentElement;
            }
        }

        public class PasswordDigestMessageInspector : IClientMessageInspector
        {
            public string Username { get; set; }
            public string Password { get; set; }
            public double DeviceTimeOffset { get; set; }

            public PasswordDigestMessageInspector(string username, string password)
            {
                this.Username = username;
                this.Password = password;
            }

            /// <summary>
            /// Create password digest message inspector to insert SOAP security
            /// header into the HTTP request (allows for time offset corrections 
            /// in case device time and client time aren't synchronized)
            /// </summary>
            /// <param name="username">User</param>
            /// <param name="password">Password</param>
            /// <param name="deviceTimeOffset">Difference between client time and device time (seconds)</param>
            public PasswordDigestMessageInspector(string username, string password, double deviceTimeOffset)
            {
                this.Username = username;
                this.Password = password;
                this.DeviceTimeOffset = deviceTimeOffset;
            }

            #region IClientMessageInspector Members

            public void AfterReceiveReply(ref System.ServiceModel.Channels.Message reply, object correlationState)
            {
                Debug.Print(string.Format("PasswordDigestMessageInspector AfterReceiveReply: {0}", reply.ToString()));
                //throw new NotImplementedException();
            }

            public object BeforeSendRequest(ref System.ServiceModel.Channels.Message request, System.ServiceModel.IClientChannel channel)
            {
                // Use a custom security token class (from Microsoft.ServiceModel.Samples.CustomToken)
                UsernameToken token = new UsernameToken(new UsernameInfo(Username, Password), DeviceTimeOffset);

                // Serialize the token to XML
                XmlElement securityToken = token.GetXml(new XmlDocument());

                // Add header to the request
                MessageHeader securityHeader = MessageHeader.CreateHeader("Security", "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd", securityToken, mustUnderstand: true);
                request.Headers.Add(securityHeader);

                // complete
                return Convert.DBNull;
            }

            #endregion
        }

        public class UsernameInfo
        {
            string _userName;
            string _password;

            public UsernameInfo(string userName, string password)
            {
                this._userName = userName;
                this._password = password;
            }

            public string Username { get { return _userName; } }
            public string Password { get { return _password; } }
        }

        public class PasswordDigestBehavior : IEndpointBehavior
        {

            public string Username { get; set; }
            public string Password { get; set; }
            public double DeviceTimeOffset { get; set; }

            public PasswordDigestBehavior(string username, string password)
            {
                this.Username = username;
                this.Password = password;
            }

            /// <summary>
            /// Create password digest behavior that allows for time offset corrections 
            /// in case device time and client time aren't synchronized.  
            /// </summary>
            /// <param name="username">User</param>
            /// <param name="password">Password</param>
            /// <param name="deviceTimeOffset">Difference between client UTC time and device UTC time in seconds (i.e. ((System.DateTime.UtcNow) - DeviceDateTime).TotalSeconds</param>
            public PasswordDigestBehavior(string username, string password, double deviceTimeOffset)
            {
                this.Username = username;
                this.Password = password;
                this.DeviceTimeOffset = deviceTimeOffset;
            }

            #region IEndpointBehavior Members

            public void AddBindingParameters(ServiceEndpoint endpoint, System.ServiceModel.Channels.BindingParameterCollection bindingParameters)
            {
                Debug.Print("PasswordDigestBehavior AddBindingParameters");
            }

            public void ApplyClientBehavior(ServiceEndpoint endpoint, System.ServiceModel.Dispatcher.ClientRuntime clientRuntime)
            {
                clientRuntime.ClientMessageInspectors.Add(new PasswordDigestMessageInspector(this.Username, this.Password, this.DeviceTimeOffset));
            }

            public void ApplyDispatchBehavior(ServiceEndpoint endpoint, System.ServiceModel.Dispatcher.EndpointDispatcher endpointDispatcher)
            {
                throw new NotImplementedException();
            }

            public void Validate(ServiceEndpoint endpoint)
            {
                Debug.Print("PasswordDigestBehavior Validate");
            }

            #endregion
        }

    }
    
}
