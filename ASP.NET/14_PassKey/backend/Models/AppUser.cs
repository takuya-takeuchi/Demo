namespace Demo.Models
{

    public sealed class AppUser
    {

        #region Properties

        public string Id { get; set; } = default!;

        public string? Email { get; set; }

        public string? DisplayName { get; set; }

        public DateTime CreatedAt { get; set; } = DateTime.UtcNow;
        public DateTime LastSeenAt { get; set; } = DateTime.UtcNow;

        public List<TodoItem> Todos { get; set; } = [];

        #endregion

    }

}
