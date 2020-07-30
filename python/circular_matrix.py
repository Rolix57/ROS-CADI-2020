# -*- coding: utf-8 -*-
"""
Created on Tue Jun  2 22:25:50 2020

@author: broke
"""

z = 1
TOP = 0
BOTTOM = 4
LEFT = 0
RIGHT = 4
MAT = [[0 for i in range(5)] for j in range(5)]

while z <= 25:
    for i in range(LEFT, RIGHT+1):
        MAT[TOP][i] = z
        z += 1
    TOP += 1
    
    for i in range(TOP, BOTTOM+1):
        MAT[i][RIGHT] = z
        z += 1
    RIGHT -= 1
    
    for i in range(RIGHT, LEFT-1, -1):
        MAT[BOTTOM][i] = z
        z += 1
    BOTTOM -= 1
    
    for i in range(BOTTOM, TOP-1, -1):
        MAT[i][LEFT] = z
        z += 1
    LEFT += 1
    
    print()
    for m in MAT:
        print(m)