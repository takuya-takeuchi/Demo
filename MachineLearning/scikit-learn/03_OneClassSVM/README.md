# OneClassSVM Classifier

## Abstracts

* How to use OneClassSVM
  * You can use custom dataset

## Requirements

* Python 3.7 or later

## Dependencies

* scikit-image
* scikit-learn

## How to use?

At first, You can install python packages from [../requirements.txt](../requirements.txt) to prepare python environmental.

````shell
$ python -m pip install -r requirements.txt
````

### Preprocess

We can use [MVTec AD](https://www.mvtec.com/company/research/datasets/mvtec-ad).
Of course, you can use custom dataset.

Create dateset binary pickle file by using [preprocess.py](./preprocess.py).

````shell
$ python preprocess.py --dataset /home/dataset/screw/train/good \
                       --width 1024 \
                       --heght 1024
````

We assume that `/home/dataset/screw/train/good` has only normal image to train.

After this, you can see `/home/dataset/screw/train/good/1024x1024px.pkl`.

### Train

You must train and generate model file by using [../01_SGDClassifier/](../01_SGDClassifier) before this task.

````shell
$ python train.py --data /home/dataset/screw/train/good/1024x1024px.pkl \
                  --width 1024 \
                  --heght 1024 \
                  --nu 0.00001
Arguments
          model: /home/dataset/screw/train/good/1024x1024px.pkl
          width: 1024
         height: 1024
             nu: 1e-05
number of samples:  320
keys:  ['filename', 'data']
image shape:  (1024, 1024, 3)
Start Training
*
optimization finished, #iter = 0
obj = 0.000005, rho = 0.003200
nSV = 1, nBSV = 0
[LibSVM]
Finished Training
````

After this, you can see `/home/dataset/screw/train/good/1024x1024px_1e-05_saved_model.pkl`.

### Eval

````shell
$ python eval.py --model /home/dataset/screw/train/good/1024x1024px_1e-05_saved_model.pkl \
                 --width 1024 \
                 --height 1024 \
                 --image_root /home/dataset/screw/test \
                 --good good \
                 --abnormal manipulated_front scratch_head scratch_neck thread_side thread_top
Arguments
          image_root: /home/dataset/screw/test
                good: ['good']
            abnormal: ['manipulated_front']
               model: /home/dataset/screw/train/good/1024x1024px_saved_model.pkl
               width: 1024
              height: 1024
Start Evaluation
Finished Evaluation
Percentage correct:
            good:  0.0
        abnormal:  89.07563025210084
````