# SGD (Stochastic Gradient Descent) Classifier

## Abstracts

* How to use SGDClassifier
  * You can use custom dataset

## Requirements

* Python 3.6 or later

## Dependencies

* scikit-learn
* scikit-image

## How to use?

At first, You can install python packages from [../requirements.txt](../requirements.txt) to prepare python environmental.

````shell
$ python -m pip install -r requirements.txt
````

### Preprocess

Create dateset binary pickle file by using [preprocess.py](./preprocess.py).

````shell
$ python preprocess.py --dataset /home/dataset/test \
                       --width 256 \
                       --heght 256
````

We assume that `/home/dataset/test` has some directries which contain image files to train.
Each directory shall contain only 1 class image.

After this, you can see `/home/dataset/test/256x256px.pkl`.

### Train

````shell
$ python preprocess.py --data /home/dataset/test/256x256px.pkl \
                       --width 256 \
                       --heght 256
Arguments
           data: /home/dataset/test/256x256px.pkl
          width: 256
         height: 256
number of samples:  2500
keys:  ['label', 'filename', 'data']
image shape:  (256, 256, 3)
labels: ['apple' 'banana' 'grape' 'mango' 'orange' 'pear' 'pineapple' 'tangerine'
 'tomato' 'watermelon']
2000
(2000, 196608)
Start Training
Finished Training
Percentage correct:  82.2
````

After this, you can see `/home/dataset/test/256x256px_saved_model.pkl`.

### Eval

````shell
$ python preprocess.py --image /home/dataset/samples/apple_0.jpg \
                       --model /home/dataset/test/256x256px.pkl \
                       --width 256 \
                       --heght 256
Arguments
          image: /home/dataset/samples/pple_0.jpg
          model: /home/dataset/test/256x256px_saved_model.pkl
          width: 256
         height: 256
Start Evaluation
Finished Evaluation
['apple']
````