using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Microsoft.Scripting.Hosting;
using WPFPython2.ViewModels.Interfaces;

namespace WPFPython2.ViewModels
{

    public sealed class PythonWrapper : IPythonWrapper
    {

        #region フィールド

        private readonly ScriptRuntime _ScriptRuntime;

        private dynamic _PythonObject;

        private dynamic _PythonTest;

        private readonly string _Path;

        #endregion

        #region コンストラクタ

        public PythonWrapper(ScriptRuntime scriptRuntime, string path)
        {
            if (scriptRuntime == null)
                throw new ArgumentNullException(nameof(scriptRuntime));
            if (path == null)
                throw new ArgumentNullException(nameof(path));

            if (!File.Exists(path))
                throw new FileNotFoundException(path);

            this._ScriptRuntime = scriptRuntime;
            this._Path = path;
            this._PythonObject = this._ScriptRuntime.UseFile(this._Path);
            this._PythonTest = this._PythonObject.PythonTest();
        }

        #endregion

        #region メソッド

        public int[] Twice(int[] array)
        {
            var list = this._PythonTest.twice(array);
            return ((IList<object>)list).Cast<int>().ToArray();
        }

        #endregion

    }

}
