''' Visuo 3D
    v.18.03.14
    Written by Mike Cichonski
    With contributions from Tae Burque
    for the Science of Imagination Laboratory
    Carleton University

    File: collect_data.py
	This file traverses the SUN3D database and calculates
	angles and distances between object triplets in each frame.
	Requires SUN3D JSON label files in the json folder.'''

import json, sys

from io import BytesIO
import numpy as np
np.set_printoptions(threshold=sys.maxsize)
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
matplotlib.use("Agg")
from os import listdir
from os.path import join, isfile

from modules.timer import *
from modules.utils import processJSON
import os

## declare objects ---------------------------------------------- ##

            

## -------------------------------------------------------------- ##



if __name__ == "__main__":

    # allObjects  = [] # list to store all objects from all files
    
    # load json files
    jsonFiles = [f for f in listdir('json') \
                 if isfile(join('json',f))]

    while 1:
        response = '1' #menu.mainMenu()
        if response == '1': # process each json file
            response = None
            options = ['http://sun3d.cs.princeton.edu/', False, '1'] #menu.optionMenu() ['', True, '1'] #
            if options == "menu": continue # back to main menu
            currentPath = options[0]
            local =       options[1]
            plot  =       options[2]
            startA = startTimer()
            for jNum,jFile in enumerate(jsonFiles):
                startB = startTimer()
                with open(join("json",jFile)) as jData:
                    data = json.load(jData)
                    # if os.path.exists(f'data/{data["name"]}'):
                    allObjects = processJSON(data,currentPath,local,plot) # process each json file
                    # exit()
                    # print("\n** File processed in %s seconds."% str(endTimer(startB)))
                    # print("** Total: "+str(jNum+1)+ " files processed in %s seconds." % str(endTimer(startA)))
                    # print("**",len(allObjects),"total objects in %s JSON files.\n" % str(jNum+1))
                    #
                    # # create log file for all the objects in all frames in all locations
                    # logFile = open(join("data","objects.log"),"w")
                    # logFile.write("---- All Objects ----\n")
                    # logFile.write("# - ID - name - RGB - Locations - Old IDs\n")
                    # for i, o in enumerate(allObjects):
                    #     oldIDs = "N/A"
                    #     if o.oldIDs:
                    #         oldIDs = " "+str(o.oldIDs)
                    #     line = str(i+1)+" - "+str(o.ID)+" - "+str(o.getName())+" - "+ \
                    #            str(o.colour[:3])+" - "+str(len(o.frames))+" - "+oldIDs+"\n"
                    #     logFile.write(line)
                    # logFile.close()

        elif response == '2':
            print("UNAVAILABLE: Working on it...")
        else:
            sys.exit(0)
