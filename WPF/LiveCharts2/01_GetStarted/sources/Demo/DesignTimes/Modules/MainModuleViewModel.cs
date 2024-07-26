using LiveChartsCore;
using LiveChartsCore.SkiaSharpView;
using LiveChartsCore.SkiaSharpView.VisualElements;

using Demo.ViewModels.Modules.Interfaces;

namespace Demo.DesignTimes.Modules
{

    internal sealed class MainModuleViewModel : IModuleViewModel
    {

        public ISeries[] Series { get; }

        public LabelVisual Title { get; }

        public Axis[] XAxes { get; }

        public Axis[] YAxes { get; }

    }

}
