#!/usr/bin/env python


# /*********************************************************************
#  * Software License Agreement (BSD License)
#  *
#  *  Copyright (c) 2013, Institute for Artificial Intelligence,
#  *  Universität Bremen.
#  *  All rights reserved.
#  *
#  *  Redistribution and use in source and binary forms, with or without
#  *  modification, are permitted provided that the following conditions
#  *  are met:
#  *
#  *   * Redistributions of source code must retain the above copyright
#  *     notice, this list of conditions and the following disclaimer.
#  *   * Redistributions in binary form must reproduce the above
#  *     copyright notice, this list of conditions and the following
#  *     disclaimer in the documentation and/or other materials provided
#  *     with the distribution.
#  *   * Neither the name of the Institute for Artificial Intelligence,
#  *     Universität Bremen, nor the names of its contributors may be
#  *     used to endorse or promote products derived from this software
#  *     without specific prior written permission.
#  *
#  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  *  POSSIBILITY OF SUCH DAMAGE.
#  *********************************************************************/


from planning_msgs.srv import *
from planning_msgs.msg import *
import rospy


def resolve_pattern(pattern, bindings):
    used_bindings = {}
    
    for key in bindings:
        if pattern.find(key) != -1:
            used_bindings[key] = bindings[key]
    
    return used_bindings


def resolve_patterns(pattern, bindings):
    dic_used_bindings = {}
    dic_iteration_index = {}
    
    for key in bindings:
        dic_iteration_index[key] = 0
        dic_used_bindings[key] = False
    
    pattern_configurations = []
    
    run_loop = True
    while run_loop:
        current_bindings = {}
        
        for key in bindings:
            current_bindings[key] = bindings[key][dic_iteration_index[key]]
        
        used_bindings = resolve_pattern(pattern, current_bindings)
        for used_binding in used_bindings:
            dic_used_bindings[used_binding] = True
        
        pattern_configurations.append(used_bindings)
        
        dic_iteration_index[dic_iteration_index.keys()[0]] = dic_iteration_index[dic_iteration_index.keys()[0]] + 1
        for i in range(len(dic_iteration_index)):
            if dic_iteration_index[dic_iteration_index.keys()[i]] >= len(bindings[dic_iteration_index.keys()[i]]):
                if i == len(dic_iteration_index) - 1:
                    run_loop = False
                else:
                    dic_iteration_index[dic_iteration_index.keys()[i]] = 0
                    dic_iteration_index[dic_iteration_index.keys()[i + 1]] = dic_iteration_index[dic_iteration_index.keys()[i + 1]] + 1
    
    unused_bindings = []
    for key in dic_used_bindings:
        if dic_used_bindings[key] == False:
            unused_bindings.append(key)
    
    return (pattern_configurations, unused_bindings)


def unify_bindings(bindings):
    dic_bindings = {}
    
    for binding in bindings:
        if not binding.key in dic_bindings:
            dic_bindings[binding.key] = []
        
        if not binding.value in dic_bindings[binding.key]:
            dic_bindings[binding.key].append(binding.value)
    
    return dic_bindings


def make_step(type, pattern):
    step = Step()
    step.type = type
    step.pattern = pattern
    
    return step


def make_binding(key, value):
    binding = Binding()
    binding.key = key
    binding.value = value
    
    return binding


def evaluate_resolved_pattern(pattern, configuration):
    # TODO(winkler): This is hacky and only for testing purposes. Only
    # supports patterns in the form `fetch ?object` right now. Add
    # more if you want to test other patterns, but this block will be
    # removed in favor of the prototypical experience trees
    # eventually.
    if not "?location" in configuration:
        configuration["?location"] = "fridge"
    
    if not "?object" in configuration:
        configuration["?object"] = "cheese"
    
    if pattern == "fetch ?object from ?location":
        plan = Plan()
        
        step_find = make_step("achieve", "find-object ?object")
        step_find.bindings.append(make_binding("?object", configuration["?object"]))
        plan.steps.append(step_find)
        
        step_find = make_step("achieve", "navigate")
        plan.steps.append(step_find)
        
        if configuration["?location"] != "table":
            step_find = make_step("achieve", "open-location ?location")
            step_find.bindings.append(make_binding("?location", configuration["?location"]))
            plan.steps.append(step_find)
        
        step_find = make_step("achieve", "pick-object ?object")
        step_find.bindings.append(make_binding("?object", configuration["?object"]))
        plan.steps.append(step_find)
        
        if configuration["?location"] != "table":
            step_find = make_step("achieve", "close-location ?location")
            step_find.bindings.append(make_binding("?location", configuration["?location"]))
            plan.steps.append(step_find)
        
        return plan


def plan_replies(pattern, bindings):
    res = PlanningResponse()
    
    bindings = unify_bindings(bindings)
    (configurations, unused_bindings) = resolve_patterns(pattern, bindings)
    
    res.unused_bindings = unused_bindings
    
    for configuration in configurations:
        plan = evaluate_resolved_pattern(pattern, configuration)
        
        if plan:
            res.plans.append(plan)
    
    print "Returning " + str(len(res.plans)) + " plan" + ("s" if len(res.plans) != 1 else "")
    
    return res


def handle_planning_request(req):
    if req.pattern != "":
        print "Planning for pattern '" + req.pattern + "'"
        
        if len(req.bindings) > 0:
            for binding in req.bindings:
                print " - " + binding.key + " = '" + binding.value + "'"
        else:
            print "No bindings defined"
        
        return plan_replies(req.pattern, req.bindings)
    else:
        print "Empty pattern, returning zero reply (aka no plans inside)."
        return PlanningResponse()


def planning_server():
    rospy.init_node('planning_server')
    s = rospy.Service('/planning_server/plan', Planning, handle_planning_request)
    print "Ready to plan"
    rospy.spin()


if __name__ == "__main__":
    planning_server()
