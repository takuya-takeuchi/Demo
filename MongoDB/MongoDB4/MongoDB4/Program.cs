using System;
using MongoDB.Bson;
using MongoDB.Driver;

namespace MongoDB4
{
    class Program
    {

        private const string UserName = "admin";

        private const string Password = "password";

        static void Main()
        {
            var mongoClient = new MongoClient(string.Format("mongodb://{0}:{1}@localhost", UserName, Password));
            //TestConnect(mongoClient);

            InsertData(mongoClient);
            TestConnect(mongoClient);

            Console.WriteLine("Please enter any key...");
            Console.ReadKey();
        }

        private static void InsertData(MongoClient mongoClient)
        {
            Console.WriteLine("InsertData");

            var database = mongoClient.GetDatabase("Database1");

            var c1 = new Company
            {
                Id = ObjectId.GenerateNewId(),
                Name = "Microsoft",
                Established = new DateTime(1975, 4, 4)
            };

            var c2 = new Company
            {
                Id = ObjectId.GenerateNewId(),
                Name = "Apple",
                Established = new DateTime(1976, 4, 1)
            };

            var collection = database.GetCollection<Company>("Companies");
            collection.InsertOne(c1);
            collection.InsertOne(c2);

            var findFluent = collection.Find(FilterDefinition<Company>.Empty);
            var count = findFluent.Count();
            Console.WriteLine("Count:{0}", count);
        }

        public sealed class Company
        {

            public ObjectId Id
            {
                get;
                set;
            }

            public string Name
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

        private static void TestConnect(MongoClient mongoClient)
        {
            Console.WriteLine("TestConnect");

            foreach (var document in mongoClient.ListDatabases().ToList())
            {
                Console.WriteLine("\tDocument:{0}", document);

                foreach (var name in document.Names)
                {
                    Console.WriteLine("\t\tName:{0}", name);
                }
            }
        }

    }
}
