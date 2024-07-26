using LiveChartsCore;
using LiveChartsCore.SkiaSharpView;
using LiveChartsCore.SkiaSharpView.VisualElements;

namespace Demo.ViewModels.Modules.Interfaces
{

    internal interface IModuleViewModel
    {

        ISeries[] Series { get; }

        LabelVisual Title { get; }

        Axis[] XAxes { get; }

        Axis[] YAxes { get; }

    }

}