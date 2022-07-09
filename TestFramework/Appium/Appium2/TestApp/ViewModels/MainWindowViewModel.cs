using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Documents;
using System.Windows.Input;
using Prism.Mvvm;
using Prism.Commands;
using TestApp.ViewModels.Interfaces;

namespace TestApp.ViewModels
{

    public sealed class MainWindowViewModel : BindableBase, IMainWindowViewModel
    {

        #region Fields
        #endregion

        #region Constructors

        public MainWindowViewModel()
        {
            this._GeneratePrimeNumberCommand = new Lazy<ICommand>(this.GeneratePrimeNumberFactory);
            this._GenerateNonPrimeNumberCommand = new Lazy<ICommand>(this.GenerateNonPrimeNumberFactory);
        }

        #endregion

        #region Properties

        private string _Title = "Prime Number Check Application";

        public string Title
        {
            get => this._Title;
            set => SetProperty(ref this._Title, value);
        }

        private Lazy<ICommand> _GeneratePrimeNumberCommand;

        public ICommand GeneratePrimeNumberCommand => this._GeneratePrimeNumberCommand.Value;

        private Lazy<ICommand> _GenerateNonPrimeNumberCommand;

        public ICommand GenerateNonPrimeNumberCommand => this._GenerateNonPrimeNumberCommand.Value;

        private ObservableCollection<uint> _Numbers;

        public ObservableCollection<uint> Numbers => this._Numbers;

        #endregion

        #region Methods

        #region Helpers

        private ICommand GeneratePrimeNumberFactory()
        {
            return new DelegateCommand(async () =>
            {
                var ret = await this.GeneratePrimeNumber();
                this._Numbers.Clear();
                foreach (var r in ret) this._Numbers.Add(r);
            });
        }

        private ICommand GenerateNonPrimeNumberFactory()
        {
            return new DelegateCommand(async () =>
            {
                var ret = await this.GeneratePrimeNumber();
                this._Numbers.Clear();
                foreach (var r in ret) this._Numbers.Add(r);
                this._Numbers.Insert(this._Numbers.Count / 2, 500);
            });
        }

        private async Task<IList<uint>> GeneratePrimeNumber()
        {
            const uint max = 1000;
            var primeNumbers = new List<uint>();
            for (var i = 2u; i <= max; i++)
                primeNumbers.Add(i);

            var p = new List<uint>();
            var s = new List<uint>();

            for (var i = 2u; i <= max; i++)
                s.Add(i);

            while (s[0] * s[0] <= max)
            {
                var prime = s[0];
                p.Add(prime);
                s.RemoveAt(0);
                s.RemoveAll(x => x % prime == 0);
            }

            p.AddRange(s);

            return p;
        }

        #endregion

        #endregion

    }

}