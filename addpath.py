# -*- coding: utf-8 -*-
"""
Created on Tue Aug 11 18:21:02 2020

@author: Zhaoliang
"""

import os
import sys

"""
This code has to be run first
Then you can have access to different functional code in different directories.
"""
current_dir = os.getcwd()    # obtain current directory
sys.path.append(current_dir) # add current dir to sys path

print(sys.path) # check if your current directory is added to sys path