# -*- coding: utf-8 -*-
"""
Created on Tue Mar  3 14:37:06 2020

@author: catar
"""

def sqdlist(l):
    i=0
    while(i<len(l)):
        l[i]=(l[i])**2
        i=i+1
    return l