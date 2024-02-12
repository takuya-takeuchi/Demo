using System;
using System.Diagnostics;
using MongoDB.Bson;
using MongoDB.Driver;

namespace MongoDB5
{
    class Program
    {

        private const string UserName = "admin";

        private const string Password = "password";

        private const string DatabaseName = "Data";

        private const string CollectionName = "Data";

        private const string DatabaseLargeName = "LargeData";

        private const string CollectionLargeName = "LargeData";

        private const int LargeDataSize = 100000;

        private const int Count = 1000000;

        private const int LargeCount = 1000000;

        static void Main()
        {
            var mongoClient = new MongoClient(string.Format("mongodb://{0}:{1}@localhost", UserName, Password));

            //InsertSmallData(mongoClient);
            WriteSmallData(mongoClient);
            //WriteLargeData(mongoClient);

            Console.WriteLine("Please enter any key...");
            Console.ReadKey();
        }

        private static void WriteSmallData(MongoClient mongoClient)
        {
            DropDatabase(mongoClient);
            InsertSmallData(mongoClient);

            DropDatabase(mongoClient);
            InsertSmallDataMany(mongoClient);
            SelectData(mongoClient);
        }

        private static void WriteLargeData(MongoClient mongoClient)
        {
            DropDatabase(mongoClient);
            InsertLargeData(mongoClient);

            //DropDatabase(mongoClient);
            //InsertLargeDataMany(mongoClient);
            SelectData(mongoClient);
        }

        private static void DropDatabase(MongoClient mongoClient)
        {
            Console.WriteLine("DropDatabase");

            var sw = new Stopwatch();

            sw.Start();
            mongoClient.DropDatabase(DatabaseName);
            sw.Stop();

            Console.WriteLine("\tTime : {0} ms", sw.ElapsedMilliseconds);
        }

        private static void InsertSmallData(MongoClient mongoClient)
        {
            Console.WriteLine("InsertSmallData");

            var database = mongoClient.GetDatabase(DatabaseName);

            var sw = new Stopwatch();

            sw.Start();
            var collection = database.GetCollection<Data>(CollectionName);
            for (var i = 0; i < Count; i++)
            {
                var data = new Data
                {
                    Id = ObjectId.GenerateNewId(),
                    No = i,
                    Established = DateTime.UtcNow
                };
                collection.InsertOne(data);
            }
            sw.Stop();

            Console.WriteLine("\tTime : {0} ms", sw.ElapsedMilliseconds);
        }

        private static void InsertSmallDataMany(MongoClient mongoClient)
        {
            Console.WriteLine("InsertSmallDataMany");

            var database = mongoClient.GetDatabase(DatabaseName);

            var sw = new Stopwatch();

            sw.Start();
            var array = new Data[Count];
            var collection = database.GetCollection<Data>(CollectionName);
            for (var i = 0; i < Count; i++)
            {
                var data = new Data
                {
                    Id = ObjectId.GenerateNewId(),
                    No = i,
                    Established = DateTime.UtcNow
                };
                array[i] = data;
            }
            collection.InsertMany(array);
            sw.Stop();

            Console.WriteLine("\tTime : {0} ms", sw.ElapsedMilliseconds);
        }

        private static void InsertLargeData(MongoClient mongoClient)
        {
            Console.WriteLine("InsertLargeData");

            var database = mongoClient.GetDatabase(DatabaseLargeName);

            var sw = new Stopwatch();

            sw.Start();
            var collection = database.GetCollection<LargeData>(CollectionLargeName);
            for (var i = 0; i < LargeCount; i++)
            {
                var data = new LargeData
                {
                    Id = ObjectId.GenerateNewId(),
                    DataBytes = new byte[LargeDataSize],
                    Established = DateTime.UtcNow
                };
                collection.InsertOne(data);
            }
            sw.Stop();

            Console.WriteLine("\tTime : {0} ms", sw.ElapsedMilliseconds);
        }

        private static void InsertLargeDataMany(MongoClient mongoClient)
        {
            Console.WriteLine("InsertLargeDataMany");

            var database = mongoClient.GetDatabase(DatabaseLargeName);

            var sw = new Stopwatch();

            sw.Start();
            var array = new LargeData[LargeCount];
            var collection = database.GetCollection<LargeData>(CollectionLargeName);
            for (var i = 0; i < LargeCount; i++)
            {
                var data = new LargeData
                {
                    Id = ObjectId.GenerateNewId(),
                    DataBytes = new byte[LargeDataSize],
                    Established = DateTime.UtcNow
                };
                array[i] = data;
            }
            collection.InsertMany(array);
            sw.Stop();

            Console.WriteLine("\tTime : {0} ms", sw.ElapsedMilliseconds);
        }

        private static void SelectData(MongoClient mongoClient)
        {
            Console.WriteLine("SelectData");

            var database = mongoClient.GetDatabase(DatabaseName);
            var sw = new Stopwatch();

            sw.Start();

            var collection = database.GetCollection<Data>(CollectionName);
            var findFluent = collection.Find(FilterDefinition<Data>.Empty);
            var count = findFluent.Count();
            Console.WriteLine("Count:{0}", count);
            sw.Stop();

            Console.WriteLine("\tTime : {0} ms", sw.ElapsedMilliseconds);
        }

        public sealed class Data
        {

            public ObjectId Id
            {
                get;
                set;
            }

            public int No
            {
                set;
                get;
            }

            public DateTime Established
            {
                get;
                set;
            }

        }

        public sealed class LargeData
        {

            public ObjectId Id
            {
                get;
                set;
            }

            public byte[] DataBytes
            {
                set;
                get;
            }

            public DateTime Established
            {
                get;
                set;
            }

        }

    }
}
