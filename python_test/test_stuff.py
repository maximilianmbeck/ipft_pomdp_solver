# -*- coding: latin-1 -*-
""" This module is for testing ipft python bindings. """

from __future__ import division
import numpy as np
import math


arr1 = np.array([1, 1])
arr2 = np.array([[1], [2]])
print(arr1)
print(arr2[1][0])

print(arr2.shape)
print(arr1.reshape(2, 1))
