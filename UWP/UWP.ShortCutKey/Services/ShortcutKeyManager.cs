using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;
using Windows.System;
using Windows.UI.Xaml;
using Prism.Commands;

namespace UWP.ShortCutKey.Services
{

    public sealed class ShortcutKeyManager
    {

        #region Fields

        private static readonly IDictionary<string, ShortcutKeyData> KeyMapping = new Dictionary<string, ShortcutKeyData>();

        private static readonly IDictionary<string, IDictionary<KeyBind, string>> GroupsBinds = new Dictionary<string, IDictionary<KeyBind, string>>();

        #endregion

        #region Constructors

        private ShortcutKeyManager()
        {
            GroupsBinds.Add("MainPage", new Dictionary<KeyBind, string>());
            GroupsBinds["MainPage"].Add(new KeyBind(VirtualKey.S, true, false, false), "Save");
            GroupsBinds["MainPage"].Add(new KeyBind(VirtualKey.N, true, false, false), "New");
        }

        #region Properties

        private static readonly ShortcutKeyManager _Instance = new ShortcutKeyManager();

        public static ShortcutKeyManager Instance => _Instance;

        #endregion

        #endregion

        #region Methods

        public void Add(string keyName, DependencyObject dependencyObject, string groupName, DelegateCommand command)
        {
            if (KeyMapping.ContainsKey(keyName))
                KeyMapping.Remove(keyName);

            KeyMapping.Add(keyName, new ShortcutKeyData(dependencyObject, command));
        }

        public bool Execute(string groupName, VirtualKey key, bool control, bool shift, bool alt)
        {
            if (!GroupsBinds.TryGetValue(groupName, out var keyMapping))
                return false;

            var bind = new KeyBind(key, control, shift, alt);
            if (!keyMapping.TryGetValue(bind, out var keyName))
                return false;

            if (!KeyMapping.TryGetValue(keyName, out var data))
                return false;

            data.Command.Execute();
            return true;
        }

        #region Overrids
        #endregion

        #region Event Handlers
        #endregion

        #region Helpers
        #endregion

        #endregion

        private sealed class KeyBind : IEquatable<KeyBind>
        {

            #region Fields

            private readonly VirtualKey _Key;

            private readonly bool _Control;

            private readonly bool _Shift;

            private readonly bool _Alt;

            #endregion

            #region Constructors

            public KeyBind(VirtualKey key, bool control, bool shift, bool alt)
            {
                this._Key = key;
                this._Control = control;
                this._Shift = shift;
                this._Alt = alt;
            }

            #endregion

            #region Methods

            #region Overrids

            public bool Equals(KeyBind other)
            {
                if (ReferenceEquals(null, other)) return false;
                if (ReferenceEquals(this, other)) return true;
                return this._Key == other._Key && this._Control == other._Control && this._Shift == other._Shift && this._Alt == other._Alt;
            }

            public override bool Equals(object obj)
            {
                return ReferenceEquals(this, obj) || obj is KeyBind other && Equals(other);
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    var hashCode = (int)this._Key;
                    hashCode = (hashCode * 397) ^ this._Control.GetHashCode();
                    hashCode = (hashCode * 397) ^ this._Shift.GetHashCode();
                    hashCode = (hashCode * 397) ^ this._Alt.GetHashCode();
                    return hashCode;
                }
            }

            #endregion

            #endregion

        }

        private sealed class ShortcutKeyData
        {

            #region Fields

            private readonly DependencyObject _DependencyObject;

            #endregion

            #region Constructors

            public ShortcutKeyData(DependencyObject dependencyObject, DelegateCommand command)
            {
                this._DependencyObject = dependencyObject;
                this.Command = command;
            }

            #endregion

            #region Properties

            public DelegateCommand Command
            {
                get;
            }

            #endregion

        }

    }

}
