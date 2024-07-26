using LiveChartsCore;
using LiveChartsCore.SkiaSharpView;
using LiveChartsCore.SkiaSharpView.VisualElements;
using Prism.Commands;

namespace Demo.ViewModels.Modules.Interfaces
{

    internal interface IModuleViewModel
    {

        ISeries[] Series { get; }

        RectangularSection[] Thumbs { get; }

        LabelVisual Title { get; }

        Axis[] XAxes { get; }

        Axis[] YAxes { get; }

        DelegateCommand StartCommand { get; }

        DelegateCommand StopCommand { get; }

    }

}