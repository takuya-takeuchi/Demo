import argparse
from re import L

import torch
import torchvision
import torchvision.transforms as transforms
from pycm import ConfusionMatrix
from tqdm import tqdm
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

from models.net import Net
import utils

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pretrained", required=True, type=str)
    parser.add_argument("--label", required=True, type=str)
    parser.add_argument("--batchsize", required=True, type=int)
    return parser.parse_args()

def train(args):
    pretrained = args.pretrained
    label      = args.label
    batchsize  = args.batchsize

    # check whether gpu is available
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    # setup transform
    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
    ])

    # setup data loader for test
    testset = torchvision.datasets.CIFAR10(root='../data',
                                           train=False,
                                           download=True,
                                           transform=transform)
    testloader = torch.utils.data.DataLoader(testset,
                                             batch_size=1,
                                             shuffle=False,
                                             num_workers=2)

    # setup network
    model = Net()
    model.load_state_dict(torch.load(pretrained))
    model.to(device)
    model.eval()

    print("Start Evaluation")

    classes = utils.get_labels(label)
    class_correct = list(0. for i in range(len(classes)))
    class_total = list(0. for i in range(len(classes)))
        
    predict_vector = list()
    actual_vector = list()

    with tqdm(testloader) as pbar:
        for images, labels in pbar:                
            images = images.to(device)
            labels = labels.to(device)

            outputs = model(images)

            pred = outputs.argmax(dim=1, keepdim=True)

            # for i in range(batchsize):
            #     l = labels[i]
            #     class_correct[l] += c[i].data.cpu()
            #     class_total[l] += 1
            predict_vector.extend(pred.cpu().numpy().reshape(-1).tolist())
            actual_vector.extend(labels.cpu().numpy().reshape(-1).tolist())
    
    print("Finished Evaluation")

    print("Results")
    cm = ConfusionMatrix(actual_vector=actual_vector,
                         predict_vector=predict_vector)
    mapping = {i: classes[i] for i in range(0, len(classes))}
    cm.relabel(mapping=mapping)                       
    plt.figure()
    data = cm.matrix
    title = "Confusion matrix"
    normalize = False
    if normalize:
        title += '(Normalized)'
        data = cm.normalized_matrix
    df = pd.DataFrame(data).T.fillna(0)
    ax = sns.heatmap(df, annot=True, cmap="YlGnBu", fmt='d')
    ax.set_title(title)
    ax.set(xlabel='Predict', ylabel='Actual')
    plt.savefig('confusion_mat.png', bbox_inches='tight')
    
    print(cm)
    # for c in range(len(classes)):
    #     print(f"\t{classes[c]}: {class_correct[c] / class_total[c]}")

if __name__ == '__main__':
    # parse args
    args = get_args()
    pretrained = args.pretrained
    label      = args.label
    batchsize  = args.batchsize

    print("Arguments")
    print("     pretrained: {}".format(pretrained))
    print("          label: {}".format(label))
    print("      batchsize: {}".format(batchsize))

    train(args)
