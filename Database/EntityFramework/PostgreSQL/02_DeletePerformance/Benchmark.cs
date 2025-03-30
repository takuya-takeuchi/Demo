using System;
using System.Linq;

using BenchmarkDotNet.Attributes;

namespace Demo
{

    [RPlotExporter] 
    public class Benchmark
    {

        #region Fields

        private Context _Context = null!;

        #endregion

        #region Properties

        [Params(10, 100, 1000, 10000)]
        public int NumberOfItems { get; set; }

        #endregion

        #region Methods

        [GlobalSetup]
        public void GlobalSetup()
        {
            // Create database if table does not exist
            this._Context = new Context();
            this._Context.Database.EnsureCreated();
        }

        [IterationSetup]
        public void Setup()
        {
            // Remove all records
            this._Context.Entities.RemoveRange(this._Context.Entities);
            this._Context.SaveChanges();

            var count = this.NumberOfItems;
            for (var i = 1; i <= count; i++)
            {
                var newEntity = new Entity { Id = i, Name = $"Name_{i}", Email = $"user${i}@example.com" };
                this._Context.Entities.Add(newEntity);
            }
            this._Context.SaveChanges();
        }

        [Benchmark]
        public void RemoveWithSaveChangesEachTime()
        {
            var count = this.NumberOfItems;
            for (var i = count; i > 0; i--)
            {
                var entity = this._Context.Entities.Single(x => x.Id == i);
                this._Context.Entities.Remove(entity);
                this._Context.SaveChanges();
            }
        }

        [Benchmark]
        public void RemoveWithSaveChangeAtEnd()
        {
            var count = this.NumberOfItems;
            for (var i = count; i > 0; i--)
            {
                var entity = this._Context.Entities.Single(x => x.Id == i);
                this._Context.Entities.Remove(entity);
            }
            this._Context.SaveChanges();
        }

        [Benchmark]
        public void RemoveRangeWithSaveChange()
        {
            this._Context.Entities.RemoveRange(this._Context.Entities);
            this._Context.SaveChanges();
        }

        #endregion

    }

}