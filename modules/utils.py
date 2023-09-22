from os.path import isdir
import  random, urllib.request, re
from io import BytesIO

import matplotlib.pyplot as plt

from modules.Frame import *
from modules.object import Object

def processJSON(data, currentPath, local, plot):
    '''process each json file
    data:   the data contained in the json file
    currentPath:    the root path of the database
    local:  True if database is located locally, False if on web
    plot:   number of different views plotted per frame'''

    # start json timer
    jsonTimer = startTimer()

    # parse json data into variables
    name = data['name']
    date = data['date']
    frames = data['frames']
    objects = data['objects']
    extrinsics = data['extrinsics']
    conflictList = data['conflictList']
    fileList = data['fileList']

    print(f'name: {name}, flst: {len(fileList)}, frames: {len(frames)}')

    # store indices of the empty frames & objects
    emptyFrames = []
    emptyObjects = []
    noneEmptyFrames = []
    noneEmptyObjects = []

    for i, f in enumerate(frames):
        if not f or not f['polygon']:
            emptyFrames.append(i)
        else:
            noneEmptyFrames.append(i)


    for i, o in enumerate(objects):
        if not o:
            emptyObjects.append(i)
        else:
            noneEmptyObjects.append(i)

    # print(join(currentPath, 'data', name, 'intrinsics.txt'))
    # get camera intrinsics
    K = np.transpose(
        np.reshape(imp.readValuesFromTxt(join(currentPath, 'data', name, 'intrinsics.txt'), local), (3, 3)))
    # K = imp.readValuesFromTxt(join(currentPath, 'data', name, 'intrinsics.txt'), local)

    # get camera extrinsics //TODO check this function.
    exFile = join(currentPath, 'data', name, 'extrinsics', extrinsics)
    # if local:
        # exFile = listdir(join('data', name, 'extrinsics'))[-1]

    # else:
        # exFile = re.compile(r'[0-9]*\.txt').findall(
        #     urllib.request.urlopen(join(currentPath, 'data', name, 'extrinsics')).read().decode('utf-8')) #[-1]

    extrinsicsC2W = np.transpose(np.reshape(imp.readValuesFromTxt(exFile, local),(-1, 3, 4)), (1, 2, 0))

        # print file stats
    print("-- processing data.....", name)
    print("  -- DATE:", date)
    print("  -- # FRAMES:", len(frames))
    print("    -- actual:", len(frames) - len(emptyFrames))
    print("    -- undefined:", len(emptyFrames))
    print("  -- # OBJECTS:", len(objects))
    print("    -- actual:", len(objects) - len(emptyObjects))
    print("    -- undefined:", len(emptyObjects))

    allObjects = []  # list to store all objects from all files

    # //TODO change this to nonEmptyFrames
    for i, f in enumerate(frames):
        if i in emptyFrames:
            continue
        frameTimer = startTimer()
        sys.stdout.write("\nCalculating Frame " + str(i))
        sys.stdout.write(" coordinates...\n")
        sys.stdout.write("\tgathering frame data:");
        sys.stdout.flush()
        # get background image and depth data ----------------#
        imagePath = join(currentPath, "data", name, "image")
        depthPath = join(currentPath, "data", name, "depth")

        if local:
            imageList = listdir(imagePath)
            depthList = listdir(depthPath)
        else:
            # ---------------------------------------------#
            imageList = re.compile(r'[0-9]*\-[0-9]*\.jpg').findall(
                urllib.request.urlopen(imagePath).read().decode('utf-8'))
            newList = []
            for x in imageList:  # remove duplicates
                if x not in newList:
                    newList.append(x)
            imageList = newList

            # ---------------------------------------------#
            depthList = re.compile(r'[0-9]*\-[0-9]*\.png').findall(
                urllib.request.urlopen(depthPath).read().decode('utf-8'))
            newList = []
            for x in depthList:  # remove duplicates
                if x not in newList:
                    newList.append(x)
            depthList = newList
            # ---------------------------------------------#

        # fileNum = "0" * (7 - len(str(1 + i * 5))) + str(1 + i * 5) + "-"
        # index = [idx for idx, s in enumerate(imageList) if fileNum in s][0]
        # image = join(imagePath, imageList[index])
        image = join(imagePath, imageList[i])
        # index = [idx for idx, s in enumerate(depthList) if fileNum in s][0]
        # depth = join(depthPath, depthList[index])
        depth = join(depthPath, depthList[i])

        # for img in imageList:
        #     # fileNum = "0" * (7 - len(str(1 + i * 5))) + str(1 + i * 5) + "-"
        #     if fileNum in img:
        #         image = join(imagePath, img)
        #         break
        # for img in depthList:
        #     # fileNum = "0" * (7 - len(str(1 + i * 5))) + str(1 + i * 5) + "-"
        #     if fileNum in img:
        #         depth = join(depthPath, img)
        #         break

        if local:
            background = Image.open(image, 'r').convert('RGBA')
        else:
            background = Image.open(BytesIO(urllib.request.urlopen(image).read())).convert('RGBA')

        (width, height) = background.size

        # ---------------------------------------------------- #
        # create frame and fill with data
        currentFrame = Frame(i, width, height)
        currentFrame.loc = name
        currentFrame.background = background
        currentFrame.depthMap = imp.depthRead(depth, local)
        currentFrame.intrinsics = K
        currentFrame.extrinsics = extrinsicsC2W[:,:,i] #imp.getExtrinsics(extrinsicsC2W, i)

        exceptions = []
        conflicts = []
        polygons = {}
        for polygon in f['polygon']:
            ID = polygon['object']

            #//TODO this is weird to check the duplicates of objects and add the last one.
            exists = False
            for o in allObjects:
                if objects[ID]['name'] == o.name:
                    currentObject = o
                    currentObject.updateID(ID)
                    exists = True
                    break

            if not exists:  # new object
                currentObject = Object(ID, objects[ID]['name'])
                allObjects.append(currentObject)

            polygons[str(currentObject.getName())] = []
            currentObject.addFrame(name, currentFrame)

            for j, x in enumerate(polygon['x']):
                x = int(round(x))
                y = int(round(polygon['y'][j]))
                polygons[str(currentObject.getName())].append((x, y))
                if 0 < x <= width and 0 < y <= height:
                    if (y, x) in zip(currentFrame.row, currentFrame.col):
                        conflicts.append(str([(x, y), currentObject.getName()]))
                    else:
                        currentFrame.addData(currentObject, x, y)
                else:
                    exceptions.append(str([(x, y), currentObject.getName()]))
            currentFrame.addObject(currentObject)


        filePath = join(currentPath, 'out', name)

        try:
            makedirs(filePath)
        except OSError:
            if not isdir(filePath):
                raise
        sys.stdout.write("\t\t%s sec.\n" % str(endTimer(frameTimer)));
        sys.stdout.flush()
        currentFrame = (currentFrame.update()
                        .calculateCentroids(polygons)
                        .getAngleDistCombos()
                        .drawPolygons(polygons, filePath)
                        .process3dPoints(polygons, filePath, int(plot))
                        .export(filePath, str(i)))

        # uncomment to save exceptions and conflicts to files
        # np.savetxt(join(filePath,str(i)+'.ex'),exceptions,fmt="%s")
        # np.savetxt(join(filePath,str(i)+'.co'),conflicts,fmt="%s")
        sys.stdout.write("\ttotal time for frame %s:" % str(currentFrame.ID))
        currentFrame = None
        sys.stdout.write(" \t%s sec." % str(endTimer(frameTimer)))
        sys.stdout.flush()

    return allObjects
# END OF processJSON()





