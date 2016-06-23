#! /usr/bin/env python
# -*- coding: utf-8 -*-
import sys, numpy

categories = numpy.loadtxt(sys.argv[1], str, delimiter="\t")
scores = numpy.load(sys.argv[2])
top_k = 3
prediction = zip(scores[0].tolist(), categories)
prediction.sort(cmp=lambda x, y: cmp(x[0], y[0]), reverse=True)
for rank, (score, name) in enumerate(prediction[:top_k], start=1):
    print('#%d | %s | %4.1f%%' % (rank, name, score * 100))