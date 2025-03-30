using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
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
            using var context = new Context();
            
            // Create database if table does not exist
            context.Database.EnsureCreated();

            // CREATE
            var newEntity = new Entity { Name = "John Doe", Email = "j-doe@example.com" };
            context.Entities.Add(newEntity);
            context.SaveChanges();
            Logger.Info($"Added: {newEntity.Name}, {newEntity.Email}");

            // READ
            var Entities = context.Entities.ToList();
            Logger.Info("Entity List:");
            foreach (var entity in Entities)
            {
                Logger.Info($"{entity.Id}: {entity.Name} ({entity.Email})");
            }

            // UPDATE
            var existingEntity = context.Entities.FirstOrDefault();
            if (existingEntity != null)
            {
                existingEntity.Name = "Jane Doe";
                context.SaveChanges();
                Logger.Info($"Updated: {existingEntity.Name}");
            }

            // DELETE
            var entityToDelete = context.Entities.FirstOrDefault(u => u.Name == "Jane Doe");
            if (entityToDelete != null)
            {
                context.Entities.Remove(entityToDelete);
                context.SaveChanges();
                Logger.Info($"Deleted: {entityToDelete.Name}");
            }
        }

        #endregion

    }

}
