#!/usr/bin/env python

#  Software License Agreement (BSD License)
#  
#  Copyright (c) 2013, Institute for Artificial Intelligence,
#  Universitaet Bremen.
#  All rights reserved.
#  
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#  
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Institute for Artificial Intelligence,
#     Universitaet Bremen, nor the names of its contributors may be
#     used to endorse or promote products derived from this software
#     without specific prior written permission.
#  
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.


import sys
from memorycondenser.MemoryCondenser import MemoryCondenser

from OutputPerks import *


sys.setrecursionlimit(10000)


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
