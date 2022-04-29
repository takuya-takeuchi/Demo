using FluentValidation;

namespace Demo
{

    internal sealed class OptionsValidator : AbstractValidator<Options>
    {

        #region Constructors

        public OptionsValidator()
        {
            RuleFor(x => x.HorizontalPatternSize).GreaterThan(0).WithMessage("Please specify greater than 0");
            RuleFor(x => x.VerticalPatternSize).GreaterThan(0).WithMessage("Please specify greater than 0");
            RuleFor(x => x.Size).GreaterThan(0).WithMessage("Please specify greater than 0");
            RuleFor(x => x.Count).GreaterThan(0).WithMessage("Please specify greater than 0");
        }

        #endregion

    }

}