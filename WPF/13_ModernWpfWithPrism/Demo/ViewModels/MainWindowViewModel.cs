using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;

using Prism.Mvvm;
using Prism.Commands;
using ModernWpf.Controls;

using Demo.ViewModels.Interfaces;
using Demo.Services.Interfaces;
using ModernWpf;

namespace Demo.ViewModels
{

    internal sealed class MainWindowViewModel : BindableBase, IMainWindowViewModel
    {

        #region Fields

        private readonly IDispatcherService _DispatcherService;

        #endregion

        #region Constructors

        public MainWindowViewModel(IDispatcherService dispatcherService)
        {
            this._DispatcherService = dispatcherService;
        }

        #endregion

        #region Properties
        #endregion

        #region Methods

        #region Overrids
        #endregion

        #region Event Handlers
        #endregion

        #region Helpers
        #endregion

        #endregion

        #region IMainWindowViewModel Membbers

        private DelegateCommand<SelectionChangedEventArgs> _PageListSelectionChanged;

        public DelegateCommand<SelectionChangedEventArgs> PageListSelectionChanged
        {
            get
            {
                return this._PageListSelectionChanged ??= new DelegateCommand<SelectionChangedEventArgs>((args) =>
                {
                    //if (!_ignoreSelectionChange)
                    //{
                    //    NavigateToSelectedPage();
                    //}
                });
            }
        }

        private DelegateCommand<AutoSuggestBoxQuerySubmittedEventArgs> _QuerySubmittedCommand;

        public DelegateCommand<AutoSuggestBoxQuerySubmittedEventArgs> QuerySubmittedCommand
        {
            get
            {
                return this._QuerySubmittedCommand ??= new DelegateCommand<AutoSuggestBoxQuerySubmittedEventArgs>((args) =>
                {
                    //if (args.ChosenSuggestion != null && args.ChosenSuggestion is ControlInfoDataItem)
                    //{
                    //    var pageType = (args.ChosenSuggestion as ControlInfoDataItem).PageType;
                    //    RootFrame.Navigate(pageType);
                    //}
                    //else if (!string.IsNullOrEmpty(args.QueryText))
                    //{
                    //    var item = _controlPagesData.FirstOrDefault(i => i.Title.Equals(args.QueryText, StringComparison.OrdinalIgnoreCase));
                    //    if (item != null)
                    //    {
                    //        RootFrame.Navigate(item.PageType);
                    //    }
                    //}
                });
            }
        }

        private DelegateCommand<AutoSuggestBoxTextChangedEventArgs> _QueryTextChangedCommand;

        public DelegateCommand<AutoSuggestBoxTextChangedEventArgs> QueryTextChangedCommand
        {
            get
            {
                return this._QueryTextChangedCommand ??= new DelegateCommand<AutoSuggestBoxTextChangedEventArgs>((args) =>
                {
                    //var suggestions = new List<ControlInfoDataItem>();

                    //if (args.Reason == AutoSuggestionBoxTextChangeReason.UserInput)
                    //{
                    //    var querySplit = sender.Text.Split(' ');
                    //    var matchingItems = _controlPagesData.Where(
                    //        item =>
                    //        {
                    //            // Idea: check for every word entered (separated by space) if it is in the name,  
                    //            // e.g. for query "split button" the only result should "SplitButton" since its the only query to contain "split" and "button" 
                    //            // If any of the sub tokens is not in the string, we ignore the item. So the search gets more precise with more words 
                    //            bool flag = true;
                    //            foreach (string queryToken in querySplit)
                    //            {
                    //                // Check if token is not in string 
                    //                if (item.Title.IndexOf(queryToken, StringComparison.CurrentCultureIgnoreCase) < 0)
                    //                {
                    //                    // Token is not in string, so we ignore this item. 
                    //                    flag = false;
                    //                }
                    //            }
                    //            return flag;
                    //        });
                    //    foreach (var item in matchingItems)
                    //    {
                    //        suggestions.Add(item);
                    //    }
                    //    if (suggestions.Count > 0)
                    //    {
                    //        controlsSearchBox.ItemsSource = suggestions.OrderByDescending(i => i.Title.StartsWith(sender.Text, StringComparison.CurrentCultureIgnoreCase)).ThenBy(i => i.Title);
                    //    }
                    //    else
                    //    {
                    //        controlsSearchBox.ItemsSource = new string[] { "No results found" };
                    //    }
                    //}
                });
            }
        }

        private DelegateCommand _ThemeChangeCommand;

        public DelegateCommand ThemeChangeCommand
        {
            get
            {
                return this._ThemeChangeCommand ??= new DelegateCommand(() =>
                {
                    this._DispatcherService.SafeAction(() =>
                    {
                        if (ThemeManager.Current.ActualApplicationTheme == ApplicationTheme.Dark)
                        {
                            ThemeManager.Current.ApplicationTheme = ApplicationTheme.Light;
                        }
                        else
                        {
                            ThemeManager.Current.ApplicationTheme = ApplicationTheme.Dark;
                        }
                    });
                });
            }
        }

        public string WindowTitle => "ModernWpfUI with Prism";

        #endregion


    }

}