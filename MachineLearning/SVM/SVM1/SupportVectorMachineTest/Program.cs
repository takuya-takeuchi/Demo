using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using LibSvm;

namespace SupportVectorMachineTest
{

    internal class Program
    {

        private static void Main()
        {
            // ランダムにデータを作成
            var r = new Random();
            const int trainCount = 500;
            const int testCount = 100;

            var temp = Path.GetTempPath();
            var trainDic = new StringBuilder();
            var testDic = new StringBuilder();
            var testDicAns = new List<int>();

            for (var l = 0; l < 10; l++)
            {
                for (var i = 0; i < trainCount; i++)
                {
                    // 0 以上 10未満の小数を生成
                    var v = r.NextDouble() + l;
                    trainDic.AppendLine($"{l} 1:{v}");
                }
                for (var i = 0; i < testCount; i++)
                {
                    // 0 以上 10未満の小数を生成
                    var v = r.NextDouble() + l;
                    testDic.AppendLine($"{l} 1:{v}");
                    testDicAns.Add(l);
                }
            }

            var tempTrainPath = Path.Combine(temp, "train");
            var tempTestPath = Path.Combine(temp, "test");

            using (var fs = new FileStream(tempTrainPath, FileMode.Create, FileAccess.Write, FileShare.Write))
            using (var sw = new StreamWriter(fs, Encoding.ASCII))
                sw.Write(trainDic.ToString());

            using (var fs = new FileStream(tempTestPath, FileMode.Create, FileAccess.Write, FileShare.Write))
            using (var sw = new StreamWriter(fs, Encoding.ASCII))
                sw.Write(testDic.ToString());

            // データセットの読み込み
            var train = Svm.ReadProblem(tempTrainPath);
            var test = Svm.ReadProblem(tempTestPath);

            // パラメータの設定
            var param = new Parameter();
            param.SvmType = SvmType.C_SVC;
            param.KernelType = KernelType.RBF;
            param.Gamma = 0.05d;
            param.C = 5d;
            param.CacheSize = 100;
            param.Degree = 3;
            param.Coef0 = 0;
            param.Nu = 0.5;
            param.Eps = 1e-3;
            param.p = 0.1;
            param.Shrinking = 1;
            param.Probability = 0;
            param.WeightLabel = new int[0];
            param.Weight = new double[0];

            var message = Svm.CheckParameter(train, param);
            if (!string.IsNullOrWhiteSpace(message))
            {
                Console.WriteLine($"Error: {message} for train problem");
                return;
            }

            message = Svm.CheckParameter(test, param);
            if (!string.IsNullOrWhiteSpace(message))
            {
                Console.WriteLine($"Error: {message} for test problem");
                return;
            }

            // 学習データとパラメータから学習
            var model = Svm.Train(train, param);

            var correct = 0;
            var total = 0;
            for (var i = 0; i < test.Count; i++)
            {
                // 学習データのベクトル
                var x = test.x[i];

                // 分類結果 (ラベルが返ってくる)
                var ret1 = (int)Svm.Predict(model, x);
                if (ret1 == testDicAns[i])
                    correct++;

                total++;
            }

            Console.WriteLine($"Accuracy: {correct / (double)total * 100}%");
        }

    }

}
