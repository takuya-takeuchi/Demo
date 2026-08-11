using Microsoft.EntityFrameworkCore;

using Demo.Models;

namespace Demo.Data
{

    public class AppDbContext(DbContextOptions<AppDbContext> options) : DbContext(options)
    {

        #region Properties

        public DbSet<AppUser> Users => Set<AppUser>();

        public DbSet<TodoItem> Todos => Set<TodoItem>();

        #endregion

        #region Methods

        protected override void OnModelCreating(ModelBuilder b)
        {
            b.Entity<AppUser>(e =>
            {
                e.HasKey(x => x.Id);
                e.Property(x => x.Id).HasMaxLength(64);
                e.Property(x => x.Email).HasMaxLength(320);
                e.Property(x => x.DisplayName).HasMaxLength(100);
            });

            b.Entity<TodoItem>(e =>
            {
                e.HasKey(x => x.Id);
                e.Property(x => x.Title).HasMaxLength(500).IsRequired();
                e.HasOne(x => x.Owner)
                 .WithMany(x => x.Todos)
                 .HasForeignKey(x => x.OwnerId)
                 .OnDelete(DeleteBehavior.Cascade);
                e.HasIndex(x => x.OwnerId);
            });
        }

        #endregion

    }

}
