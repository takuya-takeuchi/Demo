using System;

using Demo.Models;

namespace Demo.Services
{

    public sealed class MediaEventArgs : EventArgs
    {

        #region Constructors

        public MediaEventArgs(MediaAsset media)
        {
            this.Media = media;
        }

        #endregion

        #region Properties

        public MediaAsset Media { get; }

        #endregion

    }

}