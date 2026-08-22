namespace Demo.Models
{

    public sealed class TodoItem
    {

        #region Properties

        public int Id { get; set; }

        public string Title { get; set; } = default!;

        public bool IsDone { get; set; }

        public DateTime CreatedAt { get; set; } = DateTime.UtcNow;

        public string OwnerId { get; set; } = default!;

        public AppUser? Owner { get; set; }

        #endregion

    }

}
