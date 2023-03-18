using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using Demo.Models;

namespace Demo.Services.Interfaces
{

    public interface IMediaService
    {

        event EventHandler<MediaEventArgs> OnMediaAssetLoaded;

        bool IsLoading { get; }

        Task<IList<MediaAsset>> RetrieveMediaAssetsAsync(CancellationToken? token = null);

    }

}
