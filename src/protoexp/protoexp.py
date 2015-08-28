#!/usr/bin/python

import sys
from memorycondenser.MemoryCondenser import MemoryCondenser

sys.setrecursionlimit(10000)


class TextFlags:
    HAPPY = '\033[1;92m[ :) ]\033[0;97m'
    MEH = '\033[1;93m[ :| ]\033[0;97m'
    SAD = '\033[1;91m[ :( ]\033[0;97m'
    AWESOME = '\033[1;94m[ :D ]\033[0;97m'


class MCController:
    def __init__(self):
        self.mcCondenser = MemoryCondenser()
    
    def loadMemory(self, strOwl, strDesig):
        self.mcCondenser.loadExperience(strOwl, strDesig)
    
    def start(self, mode, data):
        count = self.mcCondenser.countExperiences()
        
        if count > 0:
            if mode == "experiences":
                self.mcCondenser.printExperiences(dot=not data)
            elif mode == "condensed":
                self.mcCondenser.injectExperiences(data=data)
            elif mode == "deduced":
                self.mcCondenser.injectExperiences(deduced=True, data=data)
        else:
            print TextFlags.SAD, "No experiences loaded, nothing to condense"


def printUsage():
    print TextFlags.MEH, "Usage:", sys.argv[0], "<filename.owl> (experiences | condensed | deduced) [data]"


if __name__ == "__main__":
    mcctrl = MCController()
    
    file_to_load = ""
    mode = ""
    data = False
    
    if len(sys.argv) < 3:
        printUsage()
    else:
        if len(sys.argv) >= 3:
            file_to_load = sys.argv[1]
            mode = sys.argv[2]
        
        if len(sys.argv) == 4:
            data = True
    
    if file_to_load != "":
        if (mode == "experiences" or mode == "condensed" or mode == "deduced"):
            mcctrl.loadMemory(file_to_load, "")
            mcctrl.start(mode, data)
        else:
            printUsage()
