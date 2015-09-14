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
import rospy

from planning_msgs.srv import *
from planning_msgs.msg import *


from protoexp.OutputPerks import *


def query(pattern, configuration):
    global service_client
    
    request = PlanningRequest()
    request.pattern = pattern
    
    for key in configuration:
        for value in configuration[key]:
            bdg = Binding()
            
            bdg.type = 0 # Only strings for now
            bdg.key = key
            bdg.value = value
            
            request.bindings.append(bdg)
    
    response = service_client(request)
    
    return (response.plans, response.unused_bindings)


def reconstruct_order(steps):
    root = None
    
    for step in steps:
        has_parent = False
        
        for substep in steps:
            if substep.id == step.parent:
                has_parent = True
                break
        
        if has_parent == False:
            root = step
            break
    
    tree = build_tree(root, steps)
    
    return tree


def build_tree(root, steps):
    tree = {"step": root, "children": []}
    
    for step in steps:
        if step.parent == root.id:
            tree["children"].append(build_tree(step, steps))
    
    return tree


def expand_tree(tree):
    steps = [tree["step"]]
    
    for child in tree["children"]:
        steps += expand_tree(expand_tree(child))
    
    return steps


def print_tree(tree, indentation = ""):
    pattern = tree["step"].pattern
    for cb in tree["step"].call_pattern:
        bdg = None
        
        for binding in tree["step"].bindings:
            if binding.key == cb:
                bdg = binding
                break
        
        if bdg:
            pattern += ", " + cb + " = " + bdg.value
        else:
            pattern += ", unbound " + cb
    
    print TextFlags.MEH, indentation + " -", pattern
    
    for child in tree["children"]:
        print_tree(child, indentation + "  ")


def print_plans(plans):
    for plan in plans:
        print TextFlags.HAPPY, "Plan Score =", plan.score
        
        print_tree(reconstruct_order(plan.steps))
        # for step in :
        #     pattern = step.pattern
        #     call = step.call_pattern
            
        #     for cp in step.call_pattern:
        #         for binding in step.bindings:
        #             if binding.key == cp:
        #                 pattern += " " + binding.value
            
        #     print TextFlags.HAPPY, " -", step.type, "'" + pattern + "'"


def get_prompt_input(prompt):
    sys.stdout.write(prompt + "> ")
    return sys.stdin.readline()


def print_about():
    print TextFlags.MEH, "Plan Server Interactive Client, Version 0.1.2"
    print TextFlags.MEH, "Jan Winkler <winkler@cs.uni-bremen.de>, (c) 2015"
    print TextFlags.MEH, "License: BSD (look into this file for BSD header)"


def print_help():
    print TextFlags.MEH, "Command    Effect"
    print TextFlags.MEH, "-------    ------"
    print TextFlags.MEH, "help, ?    Display this screen"
    print TextFlags.MEH, "query      Send a query to the plan server"
    print TextFlags.MEH, "about      Information about version and author"
    print TextFlags.MEH, "exit       Exit this program"


def handle_query_start():
    print TextFlags.MEH, "Usage: query> plan-name ?parameter1 ?parameter2"
    print TextFlags.MEH, "You will be asked for parameter bindings afterwards."
    print TextFlags.MEH, "If you don't want to bind a parameter, leave it empty and press <return>."
    print TextFlags.MEH, "To cancel the query, leave it empty and press <return>."
    
    call_pattern = get_prompt_input("query")
    
    if call_pattern == "":
        print ""
    
    call_pattern = call_pattern.rstrip()
    
    if call_pattern != "":
        parts = call_pattern.split(" ")
        bindings = {}
        
        print TextFlags.HAPPY, "Querying for: '" + parts[0] + "'"
        for i in range(len(parts) - 1):
            cont = True
            bindings[parts[i + 1]] = []
            
            while cont:
                bdg_cnt = len(bindings[parts[i + 1]]) + 1
                binding = get_prompt_input(str(bdg_cnt) + ("st" if bdg_cnt == 1 else ("nd" if bdg_cnt == 2 else "rd")) + " binding for '" + parts[i + 1] + "'")
                
                if binding == "":
                    print ""
                    print TextFlags.MEH, "Cancelling Query."
                    return
                
                binding = binding.rstrip()
                
                if binding == "":
                    cont = False
                else:
                    bindings[parts[i + 1]].append(binding)
        
        (plans, unused_parameters) = query(call_pattern, bindings)
        print_plans(plans)
    else:
        print TextFlags.MEH, "Cancelled Query."


def interactive_mode():
    print TextFlags.HAPPY, "Entering Interactive Mode"
    
    print TextFlags.MEH, "To query the plan server, enter 'query' and <return>."
    print TextFlags.MEH, "Then, enter the call-pattern of the plan to expand from the PE."
    print TextFlags.MEH, "You will be asked for the parameter bindings afterwards."
    print TextFlags.MEH, "Enter 'help' or '?' for instructions on how to do what."
    
    run = True
    while run:
        input_string = get_prompt_input("interactive")
        
        if input_string == "":
            run = False
            print "exit"
            exit_interactive_move()
        else:
            input_string = input_string.rstrip()
            
            if input_string == "help" or input_string == "?":
                print_help()
            elif input_string == "query":
                handle_query_start()
            elif input_string == "about":
                print_about()
            elif input_string == "exit":
                run = False
                exit_interactive_move()


def exit_interactive_move():
    print TextFlags.SAD, "You're leaving? This makes me sad. Byebye."


def test_requests():
    print TextFlags.MEH, "Running tests"
    
    request_pattern = "fetch-and-place ?object ?destination"
    request_configurations = [{"?object": "spoon", "?destination": "fridge"},
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
    rospy.init_node('planning_interactive_client')
    print TextFlags.MEH, "Waiting for planning server service"
    rospy.wait_for_service("/planning_server/plan")
    
    global service_client
    service_client = rospy.ServiceProxy("/planning_server/plan", Planning)
    
    print TextFlags.HAPPY, "Found service and established link"
    
    interactive_mode()


if __name__ == "__main__":
    start_node()
