using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

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
                Logger.Error("There is not active containers");
                return;
            }

            foreach (var container in containers)
            {
                var names = string.Join(',', container.Names);
                Logger.Info($"CONTAINER ID: {container.ImageID}, Name: {names}");
            }
        }

        #region Event Handlers
        #endregion

        #region Helpers
        #endregion

        #endregion

    }

}
