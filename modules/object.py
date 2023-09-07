import  random

class Object:
    '''represents an object in the scene'''

    def __init__(self, ID, name):
        self.ID = ID
        self.name = name
        self.colour = self.setRandomColour()
        self.frames = {}
        self.oldIDs = []

    def updateID(self, ID):
        if ID != self.ID:
            self.oldIDs.append(self.ID)
        self.ID = ID

    def setRandomColour(self):
        R = int(random.random() * 255)
        G = int(random.random() * 255)
        B = int(random.random() * 255)

        if R < 100 and G < 100 and B < 100:
            col = random.randrange(0, 3)
            if col == 0:
                R += 100
            elif col == 1:
                G += 100
            else:
                B += 100
        return (R, G, B, 140)  # return in RGBA format (transparency hard-coded at 140)

    # get object name in format: [object,identifier]
    def getName(self):
        splitName = self.name.split(':')
        n = 0
        while n < len(splitName):
            splitName[n] = str(splitName[n].strip())
            n += 1
        if len(splitName) == 1:
            splitName.append(None)
        return splitName

    # add Frame to frames dictionary
    def addFrame(self, location, f):
        if location not in self.frames:
            self.frames[location] = []
        if f not in self.frames[location]:
            self.frames[location].append(f)