# Handmade Neural Network

## Abstacts

* Simple Neural Network for classification
  * Train Cifar10

## Requirements

* Python 3.6 or later
* PyTorch 1.10.2

Please check [requirements.txt](./requirements.txt)

## How to usage?

````cmd
$ python -m pip install -r requirements.txt
````

### Train

````cmd
$ python train.py --epoch 100 --batchsize 128
````

### Evaluation

````cmd
$ python eval.py --pretrained trained.pth --label label.txt --batchsize 128
Arguments
     pretrained: trained.pth
          label: label.txt
      batchsize: 128
Files already downloaded and verified
Start Evaluation
100%|████████████████████████████████████████████████████████████████████████████████████████████████████████| 10000/10000 [00:30<00:00, 328.87it/s]
Finished Evaluation
Results
D:\Works\OpenSource\Demo\MachineLearning\PyTorch\01_Handmade\.venv\lib\site-packages\pycm\pycm_obj.py:206: RuntimeWarning: The confusion matrix is a high dimension matrix and won't be demonstrated properly.
If confusion matrix has too many zeros (sparse matrix) you can set `sparse` flag to True in printing functions otherwise by using save_csv method to save the confusion matrix in csv format you'll have better demonstration.
  warn(CLASS_NUMBER_WARNING, RuntimeWarning)
Predict     bird        car         cat         deer        dog         frog        horse       plane       ship        truck
Actual
bird        338         7           106         160         75          129         68          87          22          8

car         9           697         21          12          5           38          14          31          54          119

cat         44          8           428         101         176         119         73          22          16          13

deer        50          8           49          550         40          143         112         29          14          5

dog         35          1           202         71          506         60          95          11          14          5

frog        24          6           55          50          21          809         19          7           6           3

horse       8           6           48          83          75          19          727         14          2           18

plane       39          21          35          47          9           23          15          660         100         51

ship        10          43          24          14          8           16          15          96          740         34

truck       5           129         32          14          17          30          54          40          45          634





Overall Statistics :

95% CI                                                            (0.59934,0.61846)
ACC Macro                                                         0.92178
ARI                                                               0.3493
AUNP                                                              0.78272
AUNU                                                              0.78272
Bangdiwala B                                                      0.39099
Bennett S                                                         0.56544
CBA                                                               0.56862
CSI                                                               0.22116
Chi-Squared                                                       31287.95541
Chi-Squared DF                                                    81
Conditional Entropy                                               1.93952
Cramer V                                                          0.58961
Cross Entropy                                                     3.35525
F1 Macro                                                          0.60402
F1 Micro                                                          0.6089
FNR Macro                                                         0.3911
FNR Micro                                                         0.3911
FPR Macro                                                         0.04346
FPR Micro                                                         0.04346
Gwet AC1                                                          0.5655
Hamming Loss                                                      0.3911
Joint Entropy                                                     5.26145
KL Divergence                                                     0.03333
Kappa                                                             0.56544
Kappa 95% CI                                                      (0.55482,0.57607)
Kappa No Prevalence                                               0.2178
Kappa Standard Error                                              0.00542
Kappa Unbiased                                                    0.56495
Krippendorff Alpha                                                0.56497
Lambda A                                                          0.56544
Lambda B                                                          0.54597
Mutual Information                                                1.35146
NIR                                                               0.1
Overall ACC                                                       0.6089
Overall CEN                                                       0.45963
Overall J                                                         (4.41372,0.44137)
Overall MCC                                                       0.56674
Overall MCEN                                                      0.59369
Overall RACC                                                      0.1
Overall RACCU                                                     0.10103
P-Value                                                           None
PPV Macro                                                         0.61226
PPV Micro                                                         0.6089
Pearson C                                                         0.87052
Phi-Squared                                                       3.1288
RCI                                                               0.40683
RR                                                                1000.0
Reference Entropy                                                 3.32193
Response Entropy                                                  3.29098
SOA1(Landis & Koch)                                               Moderate
SOA2(Fleiss)                                                      Intermediate to Good
SOA3(Altman)                                                      Moderate
SOA4(Cicchetti)                                                   Fair
SOA5(Cramer)                                                      Relatively Strong
SOA6(Matthews)                                                    Moderate
Scott PI                                                          0.56495
Standard Error                                                    0.00488
TNR Macro                                                         0.95654
TNR Micro                                                         0.95654
TPR Macro                                                         0.6089
TPR Micro                                                         0.6089
Zero-one Loss                                                     3911

Class Statistics :

Classes                                                           bird          car           cat           deer          dog           frog          horse         plane         ship          truck
ACC(Accuracy)                                                     0.9114        0.9468        0.8856        0.8998        0.908         0.9232        0.9262        0.9323        0.9467        0.9378
AGF(Adjusted F-score)                                             0.58966       0.82763       0.63309       0.71455       0.69696       0.85314       0.82187       0.79718       0.84648       0.78976
AGM(Adjusted geometric mean)                                      0.76405       0.89541       0.77678       0.8228        0.81668       0.90128       0.88622       0.87545       0.90515       0.87328
AM(Difference between automatic and manual classification)        -438          -74           0             102           -68           386           192           -3            13            -110
AUC(Area under the ROC curve)                                     0.65656       0.83578       0.68222       0.74433       0.72933       0.87244       0.83767       0.81128       0.85483       0.80278
AUCI(AUC value interpretation)                                    Fair          Very Good     Fair          Good          Good          Very Good     Very Good     Very Good     Very Good     Very Good
AUPR(Area under the PR curve)                                     0.46971       0.72485       0.428         0.52455       0.52446       0.69635       0.66845       0.66099       0.73525       0.67318
BCD(Bray-Curtis dissimilarity)                                    0.0219        0.0037        0.0           0.0051        0.0034        0.0193        0.0096        0.00015       0.00065       0.0055
BM(Informedness or bookmaker informedness)                        0.31311       0.67156       0.36444       0.48867       0.45867       0.74489       0.67533       0.62256       0.70967       0.60556
CEN(Confusion entropy)                                            0.60057       0.34406       0.61748       0.54428       0.51041       0.40003       0.42241       0.43584       0.35258       0.40543
DOR(Diagnostic odds ratio)                                        20.00356      88.10565      11.02494      18.70531      20.61567      61.83098      48.879        49.90033      90.98309      59.16684
DP(Discriminant power)                                            0.71734       1.07234       0.57469       0.70127       0.72456       0.98754       0.93126       0.93621       1.08003       0.977
DPI(Discriminant power interpretation)                            Poor          Limited       Poor          Poor          Poor          Poor          Poor          Poor          Limited       Poor
ERR(Error rate)                                                   0.0886        0.0532        0.1144        0.1002        0.092         0.0768        0.0738        0.0677        0.0533        0.0622
F0.5(F0.5 score)                                                  0.52032       0.74086       0.428         0.50851       0.53511       0.61812       0.6302        0.66159       0.73238       0.69518
F1(F1 score - harmonic mean of precision and sensitivity)         0.43278       0.72378       0.428         0.52331       0.52381       0.67812       0.66332       0.66099       0.73522       0.6709
F2(F2 score)                                                      0.37045       0.70747       0.428         0.539         0.51298       0.75102       0.70012       0.6604        0.73808       0.64826
FDR(False discovery rate)                                         0.39858       0.2473        0.572         0.50091       0.45708       0.41631       0.3901        0.33801       0.2695        0.28764
FN(False negative/miss/type 2 error)                              662           303           572           450           494           191           273           340           260           366
FNR(Miss rate or false negative rate)                             0.662         0.303         0.572         0.45          0.494         0.191         0.273         0.34          0.26          0.366
FOR(False omission rate)                                          0.07014       0.03339       0.06356       0.05057       0.05448       0.02217       0.03099       0.03777       0.02893       0.04018
FP(False positive/type 1 error/false alarm)                       224           229           572           552           426           577           465           337           273           256
FPR(Fall-out or false positive rate)                              0.02489       0.02544       0.06356       0.06133       0.04733       0.06411       0.05167       0.03744       0.03033       0.02844
G(G-measure geometric mean of precision and sensitivity)          0.45087       0.72431       0.428         0.52393       0.52413       0.68717       0.66588       0.66099       0.73524       0.67204
GI(Gini index)                                                    0.31311       0.67156       0.36444       0.48867       0.45867       0.74489       0.67533       0.62256       0.70967       0.60556
GM(G-mean geometric mean of specificity and sensitivity)          0.5741        0.82418       0.63309       0.71852       0.6943        0.87013       0.83032       0.79705       0.84709       0.78484
IBA(Index of balanced accuracy)                                   0.1196        0.49073       0.19701       0.31561       0.26673       0.66106       0.53684       0.44308       0.55276       0.40804
ICSI(Individual classification success index)                     -0.06058      0.4497        -0.144        0.04909       0.04892       0.39269       0.3369        0.32199       0.4705        0.34636
IS(Information score)                                             2.58838       2.91207       2.09761       2.31931       2.44074       2.54521       2.60857       2.7268        2.86889       2.83261
J(Jaccard index)                                                  0.27614       0.56713       0.27226       0.35438       0.35484       0.513         0.49625       0.49364       0.5813        0.50478
LS(Lift score)                                                    6.01423       7.527         4.28          4.99093       5.42918       5.83694       6.09899       6.61986       7.30503       7.1236
MCC(Matthews correlation coefficient)                             0.40786       0.69502       0.36444       0.46816       0.47332       0.64674       0.62526       0.62339       0.70561       0.638
MCCI(Matthews correlation coefficient interpretation)             Weak          Moderate      Weak          Weak          Weak          Moderate      Moderate      Moderate      Strong        Moderate
MCEN(Modified confusion entropy)                                  0.70534       0.4719        0.72496       0.66941       0.62374       0.53548       0.5618        0.5807        0.49116       0.54006
MK(Markedness)                                                    0.53128       0.71931       0.36444       0.44852       0.48844       0.56152       0.5789        0.62422       0.70157       0.67218
N(Condition negative)                                             9000          9000          9000          9000          9000          9000          9000          9000          9000          9000
NLR(Negative likelihood ratio)                                    0.6789        0.31091       0.61082       0.4794        0.51854       0.20408       0.28787       0.35323       0.26813       0.37672
NLRI(Negative likelihood ratio interpretation)                    Negligible    Poor          Negligible    Poor          Negligible    Poor          Poor          Poor          Poor          Poor
NPV(Negative predictive value)                                    0.92986       0.96661       0.93644       0.94943       0.94552       0.97783       0.96901       0.96223       0.97107       0.95982
OC(Overlap coefficient)                                           0.60142       0.7527        0.428         0.55          0.54292       0.809         0.727         0.66199       0.74          0.71236
OOC(Otsuka-Ochiai coefficient)                                    0.45087       0.72431       0.428         0.52393       0.52413       0.68717       0.66588       0.66099       0.73524       0.67204
OP(Optimized precision)                                           0.42621       0.78075       0.51296       0.63872       0.60178       0.85048       0.79409       0.74583       0.81237       0.72756
P(Condition positive or support)                                  1000          1000          1000          1000          1000          1000          1000          1000          1000          1000
PLR(Positive likelihood ratio)                                    13.58036      27.39301      6.73427       8.96739       10.69014      12.61872      14.07097      17.62611      24.3956       22.28906
PLRI(Positive likelihood ratio interpretation)                    Good          Good          Fair          Fair          Good          Good          Good          Good          Good          Good
POP(Population)                                                   10000         10000         10000         10000         10000         10000         10000         10000         10000         10000
PPV(Precision or positive predictive value)                       0.60142       0.7527        0.428         0.49909       0.54292       0.58369       0.6099        0.66199       0.7305        0.71236
PRE(Prevalence)                                                   0.1           0.1           0.1           0.1           0.1           0.1           0.1           0.1           0.1           0.1
Q(Yule Q - coefficient of colligation)                            0.90478       0.97755       0.83368       0.8985        0.90747       0.96817       0.9599        0.96071       0.97826       0.96676
QI(Yule Q interpretation)                                         Strong        Strong        Strong        Strong        Strong        Strong        Strong        Strong        Strong        Strong
RACC(Random accuracy)                                             0.00562       0.00926       0.01          0.01102       0.00932       0.01386       0.01192       0.00997       0.01013       0.0089
RACCU(Random accuracy unbiased)                                   0.0061        0.00927       0.01          0.01105       0.00933       0.01423       0.01201       0.00997       0.01013       0.00893
TN(True negative/correct rejection)                               8776          8771          8428          8448          8574          8423          8535          8663          8727          8744
TNR(Specificity or true negative rate)                            0.97511       0.97456       0.93644       0.93867       0.95267       0.93589       0.94833       0.96256       0.96967       0.97156
TON(Test outcome negative)                                        9438          9074          9000          8898          9068          8614          8808          9003          8987          9110
TOP(Test outcome positive)                                        562           926           1000          1102          932           1386          1192          997           1013          890
TP(True positive/hit)                                             338           697           428           550           506           809           727           660           740           634
TPR(Sensitivity, recall, hit rate, or true positive rate)         0.338         0.697         0.428         0.55          0.506         0.809         0.727         0.66          0.74          0.634
Y(Youden index)                                                   0.31311       0.67156       0.36444       0.48867       0.45867       0.74489       0.67533       0.62256       0.70967       0.60556
dInd(Distance index)                                              0.66247       0.30407       0.57552       0.45416       0.49626       0.20147       0.27785       0.34206       0.26176       0.3671
sInd(Similarity index)                                            0.53156       0.78499       0.59305       0.67886       0.64909       0.85754       0.80353       0.75813       0.81491       0.74042
````