#!/usr/bin/env python3

import numpy as np
matrix = np.loadtxt(open("q_matrix.csv", "rb"), delimiter=",", skiprows=0)
print(matrix)