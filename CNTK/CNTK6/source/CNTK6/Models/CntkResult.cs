namespace CNTK6.Models
{

    public sealed class CntkResult
    {

        public string Label
        {
            get;
            set;
        }

        public double Probability
        {
            get;
            set;
        }

        public string ProbabilityText
        {
            get;
            set;
        }

    }

}
