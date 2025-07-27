using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;
using System.Text;

using Newtonsoft.Json;
using Newtonsoft.Json.Serialization;

using NLog;

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
            Logger.Info("Start");

            var obj = new Sample();
            obj.Title = "Title";
            obj.Number = 1024;
            obj.List = new List<KeyValuePair<string, string>> { new KeyValuePair<string, string>("Key", "Value") };

            var binaryPath = SerializeByBinaryFormatterSerialize(obj);
            var newtonsonJsonPath = SerializeByNewtonsoftJsonSerialize(obj);
            var systemJsonPath = SerializeBySystemJsonSerialize(obj);

            var binaryObj = SerializeByBinaryFormatterDeserialize<Sample>(binaryPath);
            var newtonsonJsonObj = SerializeByNewtonsoftJsonDeserialize<Sample>(newtonsonJsonPath, true);
            var newtonsonJsonObj2 = SerializeByNewtonsoftJsonDeserialize<Sample>(newtonsonJsonPath, false);
            var systemJsonObj = SerializeBySystemJsonDeserialize<Sample>(systemJsonPath);

            Logger.Info($"                      BinaryFormatter.List.Count: {binaryObj.List.Count}");
            Logger.Info($"    NewtonsoftJson.List.Count (Used Constractor): {newtonsonJsonObj.List.Count}");
            Logger.Info($"NewtonsoftJson.List.Count (Not used Constractor): {newtonsonJsonObj2.List.Count}");
            Logger.Info($"                           SystemJson.List.Count: {systemJsonObj.List.Count}");
        }

        #region Event Handlers
        #endregion

        #region Helpers

        private static T SerializeByBinaryFormatterDeserialize<T>(string path)
            where T : class
        {
            using var memoryStream = new MemoryStream(File.ReadAllBytes(path));
            var binaryFormtter = new BinaryFormatter();
            return binaryFormtter.Deserialize(memoryStream) as T;
        }

        private static string SerializeByBinaryFormatterSerialize<T>(T obj)
            where T : class
        {
            var path = "BinaryFormatter.dat";

            using var memoryStream = new MemoryStream();
            var binaryFormtter = new BinaryFormatter();
            binaryFormtter.Serialize(memoryStream, obj);
            File.WriteAllBytes(path, memoryStream.ToArray());

            return path;
        }

        private static T SerializeByNewtonsoftJsonDeserialize<T>(string path, bool useConstructor)
            where T : class
        {
            var setting = new JsonSerializerSettings()
            {
                Formatting = Formatting.Indented,
                ContractResolver = useConstructor ? null : new UninitializedObjectResolver()
            };

            return JsonConvert.DeserializeObject<T>(File.ReadAllText(path), setting);
        }

        private static string SerializeByNewtonsoftJsonSerialize<T>(T obj)
            where T : class
        {
            var path = "NewtonsoftJson.json";
            var setting = new JsonSerializerSettings()
            {
                Formatting = Formatting.Indented
            };

            var json = JsonConvert.SerializeObject(obj, setting);
            File.WriteAllText(path, json);

            return path;
        }

        private static T SerializeBySystemJsonDeserialize<T>(string path)
            where T : class
        {
            var setting = new System.Text.Json.JsonSerializerOptions()
            {
                WriteIndented = true
            };

            return System.Text.Json.JsonSerializer.Deserialize<T>(File.ReadAllText(path), setting);
        }

        private static string SerializeBySystemJsonSerialize<T>(T obj)
            where T : class
        {
            var path = "System.json";
            var setting = new System.Text.Json.JsonSerializerOptions()
            {
                WriteIndented = true
            };

            var json = System.Text.Json.JsonSerializer.Serialize(obj, setting);
            File.WriteAllText(path, json);

            return path;
        }

        #endregion

        #endregion

    }

    [DataContract]
    [Serializable]
    internal sealed class Sample
    {

        #region Constructors

        public Sample():
            this(new KeyValuePair<string, string>())
        {
        }

        public Sample(KeyValuePair<string, string> item)
        {
            this.Title = "";
            this.Number = 0;
            this.List = new List<KeyValuePair<string, string>> { item };
        }

        #endregion

        #region Properties

        [DataMember]
        public string Title
        {
            get; set;
        }

        [DataMember]
        public int Number
        {
            get; set;
        }

        [DataMember]
        public List<KeyValuePair<string, string>> List
        {
            get; set;
        }

        #endregion

    }

    internal sealed class UninitializedObjectResolver : DefaultContractResolver
    {
        protected override JsonObjectContract CreateObjectContract(Type objectType)
        {
            var contract = base.CreateObjectContract(objectType);

            contract.DefaultCreatorNonPublic = true;
            contract.DefaultCreator = () =>
            {
                return FormatterServices.GetUninitializedObject(objectType);
            };

            return contract;
        }
    }

}
