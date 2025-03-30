using System;
using Microsoft.EntityFrameworkCore;

namespace Demo
{
    internal sealed class Context : DbContext
    {

        #region Properties

        public DbSet<Entity> Entities { get; set; }

        #endregion

        #region Methods

        #region Overrides

        protected override void OnConfiguring(DbContextOptionsBuilder optionsBuilder)
        {
            optionsBuilder.UseNpgsql("Host=localhost;Port=5432;Database=demo;Username=postgres;Password=postgres");
        }
        
        protected override void OnModelCreating(ModelBuilder modelBuilder)
        {
            modelBuilder.Entity<Entity>(entity =>
            {
                entity.ToTable("id");
                entity.HasKey(e => e.Id);

                entity.Property(e => e.Name)
                .HasColumnName("name");

                entity.Property(e => e.Email)
                .HasColumnName("email");
            });

            base.OnModelCreating(modelBuilder);
        }

        #endregion

        #endregion

    }

}