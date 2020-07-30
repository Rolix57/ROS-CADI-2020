# -*- coding: utf-8 -*-
"""
Created on Tue Jun  2 21:26:22 2020

@author: broke
"""

MAT = [[7, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0],
       [0, 0, -3, 0, 9, 0],
       [0, 0, 0, 0, 0, 0],
       [0, 0, -1, 0, 0, 0],
       [0, -6, 0, 0, -5, 1]]

VALUES = list()
ROWC = list()
non_zero_elements = 0
COL = list()

for m in MAT:
    i = 0
    for e in m:
        if e != 0:
            non_zero_elements += 1
            VALUES.append(e)
            COL.append(i)
        i += 1
    ROWC.append(non_zero_elements)
            
print(VALUES)
print(ROWC)
print(COL)