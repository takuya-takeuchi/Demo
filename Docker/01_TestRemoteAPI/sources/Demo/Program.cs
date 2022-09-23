using Docker.DotNet;
using Docker.DotNet.Models;
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
            if (args.Length != 1)
            {
                Logger.Error($"{nameof(Demo)} <docker remote api endpoint>");
                return;
            }

            var client = new DockerClientConfiguration(new Uri(args[0])).CreateClient();
            var containers = client.Containers.ListContainersAsync(new ContainersListParameters
            {
                All = true
            }).Result?.ToArray() ?? Array.Empty<ContainerListResponse>();

            if (!containers.Any())
            {
                Logger.Info("There is not active containers");
                return;
            }

            foreach (var container in containers)
            {
                var names = string.Join(',', container.Names);
                Logger.Info($"CONTAINER ID: {container.ImageID}, Name: {names}");
            }
        }

        #endregion

    }

}
