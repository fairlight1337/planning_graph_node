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


import rospy

from planning_msgs.srv import *
from planning_msgs.msg import *


from OutputPerks import *


def get_plans(pattern, configuration):
    global service_client
    
    request = PlanningRequest()
    request.pattern = pattern
    
    for key in configuration:
        bdg = Binding()
        
        bdg.type = 0 # Only strings for now
        bdg.key = key
        bdg.value = configuration[key]
        
        request.bindings.append(bdg)
    
    response = service_client(request)
    
    return (response.plans, response.unused_bindings)


def print_plans(plans):
    for plan in plans:
        print TextFlags.HAPPY, "Plan Score =", plan.score
        
        for step in plan.steps:
            pattern = step.pattern
            
            for binding in step.bindings:
                pattern = pattern.replace(binding.key, binding.value)
            
            print TextFlags.HAPPY, " -", step.type, "'" + pattern + "'"


def test_requests():
    print TextFlags.MEH, "Running tests"
    
    request_pattern = "fetch-and-place ?object ?destination"
    request_configurations = [{"?object": "cheese"},
                              #{"?object": "spoon", "?destination": "drawer"},
                              #{"?destination": "table"},
                              ]#{"?object": "cheese", "?destination": "closet"}]
    
    print TextFlags.MEH, " - Pattern to test for: '" + request_pattern + "'"
    print TextFlags.MEH, " -", len(request_configurations), "parameter configuration" + ("s" if len(request_configurations) != 1 else "")  + " to test" + (":" if len(request_configurations) > 0 else "")
    for configuration in request_configurations:
        config_string = ""
        
        for key in configuration:
            config_string = config_string + (", " if config_string != "" else "") + key + " = " + configuration[key]
        
        print TextFlags.MEH, "   [", config_string, "]"
    
    for configuration in request_configurations:
        resulting_pattern = request_pattern
        for key in configuration:
            resulting_pattern = resulting_pattern.replace(key, configuration[key])
        
        print TextFlags.MEH, "Test:\033[92;1m", resulting_pattern + "\033[0m"
        (plans, unused_bindings) = get_plans(request_pattern, configuration)
        
        if len(unused_bindings) > 0:
            print TextFlags.HUH, "You're using too many parameters. Unused ones are:"
            for unused_binding in unused_bindings:
                print TextFlags.HUH, " -", unused_binding, "=", configuration[unused_binding]
        
        if len(plans) > 0:
            print TextFlags.HAPPY, "Got", len(plans), "result" + ("s" if len(plans) != 1 else "")
            print_plans(plans)
        else:
            print TextFlags.SAD, "Got no results"


def start_node():
    rospy.init_node('planning_test_client')
    print TextFlags.MEH, "Waiting for planning server service"
    rospy.wait_for_service("/planning_server/plan")
    
    global service_client
    service_client = rospy.ServiceProxy("/planning_server/plan", Planning)
    
    print TextFlags.HAPPY, "Found service and established link"
    
    test_requests()


if __name__ == "__main__":
    start_node()
