# SGD (Stochastic Gradient Descent) Classifier

## Abstracts

* How to convert saved model of SGDClassifier to onnx
* How to use onnxruntime

## Requirements

* Python 3.7 or later

## Dependencies

* onnxmltools
* onnxruntime
* scikit-image
* scikit-learn
* skl2onnx

## How to use?

At first, You can install python packages from [../requirements.txt](../requirements.txt) to prepare python environmental.

````shell
$ python -m pip install -r requirements.txt
````

### Convert

You must train and generate model file by using [../01_SGDClassifier/](../01_SGDClassifier) before this task.

````shell
$ python convert.py --data /home/dataset/test/256x256px.pkl \
                    --width 256 \
                    --heght 256
Arguments
          model: /home/dataset/test/256x256px_saved_model.pkl
          width: 256
         height: 256
Start Conversion
Finished Conversion
````

After this, you can see `/home/dataset/test/256x256px_saved_model.onnx`.

### Eval

````shell
$ python eval.py --image /home/dataset/samples/tomato_0.jpg \
                 --model /home/dataset/test/256x256px_saved_model.onnx \
                 --width 256 \
                 --heght 256
Arguments
          image: /home/dataset/samples/tomato_0.jpg
          model: /home/dataset/test/256x256px_saved_model.onnx
          width: 256
         height: 256
Start Evaluation
Finished Evaluation
[array([8], dtype=int64)]
````