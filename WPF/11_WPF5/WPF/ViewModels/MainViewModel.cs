using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Threading;
using System.Windows.Data;
using GalaSoft.MvvmLight;
using GalaSoft.MvvmLight.Command;
using WPF.Models;
using WPF.ViewModels.Interfaces;

namespace WPF.ViewModels
{

    public sealed class MainViewModel : ViewModelBase, IMainViewModel
    {

        #region Fields

        private readonly Timer _Timer;

        private GridSizeModel _SelectedGridSizeModel;

        private int _Total = 0;

        private int _Max = 0;

        #endregion

        #region Constructors

        public MainViewModel()
        {
            this._GridSizes = new[]
            {
                new GridSizeModel(2,2),
                new GridSizeModel(4,2),
                new GridSizeModel(2,4),
                new GridSizeModel(4,4),
            };
            
            this.Column = this._GridSizes.First().Column;
            this.Row = this._GridSizes.First().Row;
            this._Max = this._GridSizes.Max(model => model.Column * model.Row);

            this._ItemQueue = new ObservableCollection<ItemModel>();
            this._Items = new CollectionViewSource();
            this._Items.Source = this._ItemQueue;

            this._Timer = new Timer(state =>
            {
                this._Total++;
                App.Current.Dispatcher.BeginInvoke(new Action(() =>
                {
                    this._ItemQueue.Insert(0, new ItemModel(this._Total));
                    if (this._ItemQueue.Count > this._Max)
                        this._ItemQueue.RemoveAt(this._Max);
                }));
            }, null, new TimeSpan(0, 0, 0, 5), new TimeSpan(0, 0, 0, 1));
        }

        #endregion

        #region Properties

        private int _Column;

        public int Column
        {
            get => this._Column;
            private set
            {
                this._Column = value;
                this.RaisePropertyChanged();
            }
        }

        private readonly IEnumerable<GridSizeModel> _GridSizes;

        public IEnumerable<GridSizeModel> GridSizes => this._GridSizes;

        private readonly ObservableCollection<ItemModel> _ItemQueue;

        private readonly CollectionViewSource _Items;

        public CollectionViewSource Items => this._Items;

        private int _Row;

        public int Row
        {
            get => this._Row;
            private set
            {
                this._Row = value;
                this.RaisePropertyChanged();
            }
        }

        private RelayCommand<GridSizeModel> _SelectGridSizeCommand;

        public RelayCommand<GridSizeModel> SelectGridSizeCommand
        {
            get
            {
                return this._SelectGridSizeCommand ?? (this._SelectGridSizeCommand = new RelayCommand<GridSizeModel>(model =>
                {
                    this._SelectedGridSizeModel = model;
                    this.Column = model.Column;
                    this.Row = model.Row;
                }));
            }
        }

        #endregion

    }

}
