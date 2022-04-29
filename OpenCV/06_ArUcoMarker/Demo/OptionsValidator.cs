using FluentValidation;
using OpenCvSharp.Aruco;

namespace Demo
{

    internal sealed class OptionsValidator : AbstractValidator<Options>
    {

        #region Constructors

        public OptionsValidator()
        {
            RuleFor(x => x.Dictionary).IsEnumName(typeof(PredefinedDictionaryName)).WithMessage("Please specify proper name");
            RuleFor(x => x.Parameter).Must(File.Exists).WithMessage("Please specify valid parameter file");
            RuleFor(x => x.Size).GreaterThan(0).WithMessage("Please specify greater than 0");
        }

        #endregion

    }

}