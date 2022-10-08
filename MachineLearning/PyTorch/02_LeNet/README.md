# LeNet

## Abstracts

* LeNet for classification
  * Train MNIST

## Requirements

* Python 3.6 or later
* PyTorch 1.10.2

Please check [requirements.txt](./requirements.txt)

## How to usage?

````cmd
$ python -m pip install -r requirements.txt --extra-index-url https://download.pytorch.org/whl/cu113
````

### Train

````cmd
$ python train.py --epoch 100 --batchsize 512 --tensorboard-logdir ..\00_Tensorboard\log
````

### Evaluation

````cmd
$ python eval.py --label label.txt --batchsize 512 --pretrained trained.pth
Arguments
     pretrained: trained.pth
          label: label.txt
      batchsize: 512
Start Evaluation
100%|█████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████| 10000/10000 [00:28<00:00, 356.42it/s]
Finished Evaluation
Results
D:\Works\OpenSource\Demo\MachineLearning\PyTorch\02_LeNet\.venv\lib\site-packages\pycm\pycm_obj.py:206: RuntimeWarning: The confusion matrix is a high dimension matrix and won't be demonstrated properly.
If confusion matrix has too many zeros (sparse matrix) you can set `sparse` flag to True in printing functions otherwise by using save_csv method to save the confusion matrix in csv format you'll have better demonstration.
  warn(CLASS_NUMBER_WARNING, RuntimeWarning)
Predict    0          1          2          3          4          5          6          7          8          9
Actual
0          974        0          2          0          0          0          2          1          1          0

1          0          1131       1          1          0          0          1          0          1          0

2          1          0          1025       1          1          0          0          3          1          0

3          1          0          1          1000       0          4          0          1          3          0

4          1          1          0          0          973        0          4          0          1          2

5          2          0          0          5          0          880        2          1          1          1

6          2          2          1          0          1          2          949        0          1          0

7          0          2          1          2          0          0          0          1020       1          2

8          3          0          3          6          0          0          0          2          957        3

9          0          1          0          5          7          1          1          6          3          985





Overall Statistics :

95% CI                                                            (0.98739,0.99141)
ACC Macro                                                         0.99788
ARI                                                               0.9768
AUNP                                                              0.99411
AUNU                                                              0.99404
Bangdiwala B                                                      0.97914
Bennett S                                                         0.98822
CBA                                                               0.98684
CSI                                                               0.97864
Chi-Squared                                                       87878.91646
Chi-Squared DF                                                    81
Conditional Entropy                                               0.10641
Cramer V                                                          0.98815
Cross Entropy                                                     3.31946
F1 Macro                                                          0.98931
F1 Micro                                                          0.9894
FNR Macro                                                         0.01074
FNR Micro                                                         0.0106
FPR Macro                                                         0.00118
FPR Micro                                                         0.00118
Gwet AC1                                                          0.98822
Hamming Loss                                                      0.0106
Joint Entropy                                                     3.42583
KL Divergence                                                     3e-05
Kappa                                                             0.98822
Kappa 95% CI                                                      (0.98599,0.99045)
Kappa No Prevalence                                               0.9788
Kappa Standard Error                                              0.00114
Kappa Unbiased                                                    0.98822
Krippendorff Alpha                                                0.98822
Lambda A                                                          0.98804
Lambda B                                                          0.98804
Mutual Information                                                3.21284
NIR                                                               0.1135
Overall ACC                                                       0.9894
Overall CEN                                                       0.02443
Overall J                                                         (9.78868,0.97887)
Overall MCC                                                       0.98822
Overall MCEN                                                      0.04341
Overall RACC                                                      0.10036
Overall RACCU                                                     0.10036
P-Value                                                           None
PPV Macro                                                         0.98938
PPV Micro                                                         0.9894
Pearson C                                                         0.94754
Phi-Squared                                                       8.78789
RCI                                                               0.96789
RR                                                                1000.0
Reference Entropy                                                 3.31942
Response Entropy                                                  3.31925
SOA1(Landis & Koch)                                               Almost Perfect
SOA2(Fleiss)                                                      Excellent
SOA3(Altman)                                                      Very Good
SOA4(Cicchetti)                                                   Excellent
SOA5(Cramer)                                                      Very Strong
SOA6(Matthews)                                                    Very Strong
Scott PI                                                          0.98822
Standard Error                                                    0.00102
TNR Macro                                                         0.99882
TNR Micro                                                         0.99882
TPR Macro                                                         0.98926
TPR Micro                                                         0.9894
Zero-one Loss                                                     106

Class Statistics :

Classes                                                           0             1             2             3             4             5             6             7             8             9             
ACC(Accuracy)                                                     0.9984        0.999         0.9984        0.997         0.9982        0.9981        0.9981        0.9978        0.997         0.9968        
AGF(Adjusted F-score)                                             0.99615       0.99781       0.996         0.99339       0.99491       0.99321       0.99468       0.99501       0.99075       0.98846       
AGM(Adjusted geometric mean)                                      0.99757       0.99857       0.99747       0.99575       0.99685       0.9959        0.99671       0.9968        0.99433       0.99305       
AM(Difference between automatic and manual classification)        4             2             2             10            0             -5            1             6             -4            -16           
AUC(Area under the ROC curve)                                     0.99638       0.9979        0.99611       0.99394       0.99492       0.99289       0.99475       0.99533       0.99055       0.98766       
AUCI(AUC value interpretation)                                    Excellent     Excellent     Excellent     Excellent     Excellent     Excellent     Excellent     Excellent     Excellent     Excellent     
AUPR(Area under the PR curve)                                     0.99186       0.9956        0.99226       0.98525       0.99084       0.98933       0.99009       0.98934       0.98457       0.98408       
BCD(Bray-Curtis dissimilarity)                                    0.0002        0.0001        0.0001        0.0005        0.0           0.00025       5e-05         0.0003        0.0002        0.0008        
BM(Informedness or bookmaker informedness)                        0.99277       0.9958        0.99221       0.98787       0.98984       0.98578       0.9895        0.99066       0.98111       0.97532       
CEN(Confusion entropy)                                            0.01981       0.01135       0.01912       0.03262       0.02042       0.0242        0.02367       0.02488       0.03534       0.03482       
DOR(Diagnostic odds ratio)                                        146262.33333  417480.375    145761.50794  44850.0       108219.22222  95343.80952   95237.42222   81581.78571   39029.1448    46084.66146   
DP(Discriminant power)                                            2.84769       3.09882       2.84687       2.56465       2.77556       2.74523       2.74496       2.7079        2.53137       2.57115       
DPI(Discriminant power interpretation)                            Fair          Good          Fair          Fair          Fair          Fair          Fair          Fair          Fair          Fair          
ERR(Error rate)                                                   0.0016        0.001         0.0016        0.003         0.0018        0.0019        0.0019        0.0022        0.003         0.0032        
F0.5(F0.5 score)                                                  0.99064       0.99507       0.99168       0.98232       0.99084       0.99099       0.98978       0.98761       0.98578       0.98876       
F1(F1 score - harmonic mean of precision and sensitivity)         0.99185       0.9956        0.99226       0.98522       0.99084       0.98932       0.99009       0.98933       0.98457       0.98402       
F2(F2 score)                                                      0.99307       0.99612       0.99283       0.98814       0.99084       0.98765       0.9904        0.99106       0.98335       0.97932       
FDR(False discovery rate)                                         0.01016       0.00528       0.0087        0.01961       0.00916       0.00789       0.01043       0.01354       0.0134        0.00806       
FN(False negative/miss/type 2 error)                              6             4             7             10            9             12            9             8             17            24            
FNR(Miss rate or false negative rate)                             0.00612       0.00352       0.00678       0.0099        0.00916       0.01345       0.00939       0.00778       0.01745       0.02379       
FOR(False omission rate)                                          0.00067       0.00045       0.00078       0.00111       0.001         0.00132       0.001         0.00089       0.00188       0.00266       
FP(False positive/type 1 error/false alarm)                       10            6             9             20            9             7             10            14            13            8             
FPR(Fall-out or false positive rate)                              0.00111       0.00068       0.001         0.00222       0.001         0.00077       0.00111       0.00156       0.00144       0.00089       
G(G-measure geometric mean of precision and sensitivity)          0.99186       0.9956        0.99226       0.98523       0.99084       0.98932       0.99009       0.98933       0.98457       0.98405       
GI(Gini index)                                                    0.99277       0.9958        0.99221       0.98787       0.98984       0.98578       0.9895        0.99066       0.98111       0.97532       
GM(G-mean geometric mean of specificity and sensitivity)          0.99638       0.9979        0.9961        0.99393       0.99491       0.99287       0.99474       0.99532       0.99052       0.9876        
IBA(Index of balanced accuracy)                                   0.9878        0.99297       0.98649       0.98031       0.98176       0.97328       0.98131       0.98451       0.96542       0.95301       
ICSI(Individual classification success index)                     0.98371       0.9912        0.98451       0.97049       0.98167       0.97866       0.98018       0.97868       0.96914       0.96816       
IS(Information score)                                             3.33634       3.1316        3.26387       3.279         3.33485       3.47538       3.36871       3.26242       3.34047       3.29733       
J(Jaccard index)                                                  0.98384       0.99124       0.98463       0.97087       0.98184       0.97887       0.98037       0.97889       0.9696        0.96853       
LS(Lift score)                                                    10.10038      8.76408       9.60558       9.70685       10.08997      11.12229      10.32957      9.59592       10.12934      9.83096       
MCC(Matthews correlation coefficient)                             0.99097       0.99504       0.99136       0.98357       0.98984       0.98828       0.98904       0.98811       0.98291       0.98228       
MCCI(Matthews correlation coefficient interpretation)             Very Strong   Very Strong   Very Strong   Very Strong   Very Strong   Very Strong   Very Strong   Very Strong   Very Strong   Very Strong   
MCEN(Modified confusion entropy)                                  0.03548       0.02051       0.03429       0.05745       0.03618       0.0429        0.04223       0.04425       0.06247       0.06117       
MK(Markedness)                                                    0.98917       0.99427       0.99052       0.97928       0.98984       0.99079       0.98858       0.98557       0.98472       0.98928       
N(Condition negative)                                             9020          8865          8968          8990          9018          9108          9042          8972          9026          8991          
NLR(Negative likelihood ratio)                                    0.00613       0.00353       0.00679       0.00992       0.00917       0.01346       0.0094        0.00779       0.01748       0.02381       
NLRI(Negative likelihood ratio interpretation)                    Good          Good          Good          Good          Good          Good          Good          Good          Good          Good          
NPV(Negative predictive value)                                    0.99933       0.99955       0.99922       0.99889       0.999         0.99868       0.999         0.99911       0.99812       0.99734       
OC(Overlap coefficient)                                           0.99388       0.99648       0.99322       0.9901        0.99084       0.99211       0.99061       0.99222       0.9866        0.99194       
OOC(Otsuka-Ochiai coefficient)                                    0.99186       0.9956        0.99226       0.98523       0.99084       0.98932       0.99009       0.98933       0.98457       0.98405       
OP(Optimized precision)                                           0.99588       0.99757       0.9955        0.99314       0.9941        0.99171       0.99393       0.99467       0.98892       0.98521       
P(Condition positive or support)                                  980           1135          1032          1010          982           892           958           1028          974           1009          
PLR(Positive likelihood ratio)                                    896.47755     1472.29295    989.68562     445.0495      992.8167      1283.63869    895.70543     635.86993     682.18939     1097.14259    
PLRI(Positive likelihood ratio interpretation)                    Good          Good          Good          Good          Good          Good          Good          Good          Good          Good          
POP(Population)                                                   10000         10000         10000         10000         10000         10000         10000         10000         10000         10000         
PPV(Precision or positive predictive value)                       0.98984       0.99472       0.9913        0.98039       0.99084       0.99211       0.98957       0.98646       0.9866        0.99194       
PRE(Prevalence)                                                   0.098         0.1135        0.1032        0.101         0.0982        0.0892        0.0958        0.1028        0.0974        0.1009        
Q(Yule Q - coefficient of colligation)                            0.99999       1.0           0.99999       0.99996       0.99998       0.99998       0.99998       0.99998       0.99995       0.99996       
QI(Yule Q interpretation)                                         Strong        Strong        Strong        Strong        Strong        Strong        Strong        Strong        Strong        Strong        
RACC(Random accuracy)                                             0.00964       0.0129        0.01067       0.0103        0.00964       0.00791       0.00919       0.01063       0.00945       0.01002       
RACCU(Random accuracy unbiased)                                   0.00964       0.0129        0.01067       0.0103        0.00964       0.00791       0.00919       0.01063       0.00945       0.01002       
TN(True negative/correct rejection)                               9010          8859          8959          8970          9009          9101          9032          8958          9013          8983          
TNR(Specificity or true negative rate)                            0.99889       0.99932       0.999         0.99778       0.999         0.99923       0.99889       0.99844       0.99856       0.99911       
TON(Test outcome negative)                                        9016          8863          8966          8980          9018          9113          9041          8966          9030          9007          
TOP(Test outcome positive)                                        984           1137          1034          1020          982           887           959           1034          970           993           
TP(True positive/hit)                                             974           1131          1025          1000          973           880           949           1020          957           985           
TPR(Sensitivity, recall, hit rate, or true positive rate)         0.99388       0.99648       0.99322       0.9901        0.99084       0.98655       0.99061       0.99222       0.98255       0.97621       
Y(Youden index)                                                   0.99277       0.9958        0.99221       0.98787       0.98984       0.98578       0.9895        0.99066       0.98111       0.97532       
dInd(Distance index)                                              0.00622       0.00359       0.00686       0.01015       0.00922       0.01347       0.00946       0.00794       0.01751       0.0238        
sInd(Similarity index)                                            0.9956        0.99746       0.99515       0.99282       0.99348       0.99047       0.99331       0.99439       0.98762       0.98317
````