namespace Demo.Models
{

    public sealed class MediaAsset
    {

        #region Fields
        #endregion

        #region Constructors
        
        public MediaAsset(string id, MediaAssetType type, string name, string previewPath, string path)
        {
            this.Id = id;
            this.Type = type;
            this.Name = name;
            this.PreviewPath = previewPath;
            this.Path = path;
        }

        #endregion

        #region Properties

        public string Id { get; }

        public string Name { get; }

        public MediaAssetType Type { get; }

        public string PreviewPath { get; }

        public string Path { get; }

        #endregion

    }

}