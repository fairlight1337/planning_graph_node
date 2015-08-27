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


import os
import sys
import json
import rospy

from planning_msgs.srv import *
from planning_msgs.msg import *


loaded_datasets = []


class TextFlags:
    HAPPY = '\033[1;92m[ :) ]\033[0;97m'
    INDIFFERENT = '\033[1;93m[ :| ]\033[0;97m'
    SAD = '\033[1;91m[ :( ]\033[0;97m'
    AWESOME = '\033[1;94m[ :D ]\033[0;97m'


def process_loadable_dataset(dataset):
    steps = []
    
    for step in dataset:
        invocations_processed = []
        
        for invocation in step["invocations"]:
            invocation_processed = {}
            
            for node_uid in invocation:
                parameters = invocation[node_uid]
                node_call_processed = {}
                
                for parameter in parameters:
                    node_call_processed[param_exp_to_prolog(parameter)] = parameters[parameter]
                
                invocation_processed[node_uid] = node_call_processed
            
            invocations_processed.append(invocation_processed)
        
        call_pattern_split = step["call-pattern"].split(" ")
        call_pattern = []
        
        for call_pattern_param in call_pattern_split:
            call_pattern.append(param_exp_to_prolog(call_pattern_param))
        
        steps.append({"name": (step["node"][21:] if step["node"][:21] == "REPLACEABLE-FUNCTION-" else step["node"]).lower(),
                      "invocations": invocations_processed,
                      "call-pattern": call_pattern,
                      "uid": step["uid"]})
    
    return steps


def load_data():
    global loaded_datasets
    
    global_dir = os.path.dirname(os.path.abspath(__file__)) + "/../data"
    filenames = ["deduced_experiences.json"]
    
    if len(filenames) > 0:
        print TextFlags.INDIFFERENT, "Loading plan data:"
        for filename in filenames:
            print TextFlags.HAPPY, "- " + filename
        
        with open(global_dir + "/" + filename, "r") as f:
            datasets = json.load(f)
            
            for dataset in datasets:
                loaded_datasets.append(process_loadable_dataset(dataset))
    else:
        print TextFlags.SAD, "No plan data defined. Is this what you intended?"


def resolve_pattern(pattern, bindings):
    used_bindings = {}
    
    for key in bindings:
        if pattern.find(key) != -1:
            if not key in used_bindings:
                used_bindings[key] = []
            
            used_bindings[key].append(bindings[key])
    
    return used_bindings


def resolve_patterns(pattern, bindings):
    dic_used_bindings = {}
    dic_iteration_index = {}
    
    for key in bindings:
        dic_iteration_index[key] = 0
        dic_used_bindings[key] = False
    
    pattern_configurations = []
    
    run_loop = True if len(bindings) > 0 else False
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


def param_exp_to_prolog(param):
    return "?" + param.lower()


def param_prolog_to_exp(param):
    return param[1:].upper()


def expandCPD(dataset, configuration):
    fitting_datasets = []
    
    index = {}
    for parameter in configuration:
        index[parameter] = 0
    
    run_loop = True
    while run_loop:
        current_configuration = {}
        
        for parameter in configuration:
            current_configuration[parameter] = configuration[parameter][index[parameter]]
        
        # do
        steps = []
        all_steps_fit = True
        step0uid = dataset[0]["uid"]
        
        for step in dataset:
            step_invocable = False
            working_invocation = None
            
            for invocation in step["invocations"]:
                invocation0 = invocation[str(step0uid)]
                
                all_fit = True
                for parameter in invocation0:
                    if not invocation0[parameter] == current_configuration[parameter]:
                        all_fit = False
                
                if all_fit:
                    step_invocable = True
                    working_invocation = invocation[str(step["uid"])].copy()
            
            if step_invocable:
                step_copied = step.copy()
                step_copied["invocations"] = [working_invocation]#[current_configuration]
                steps.append(step_copied)
            else:
                all_steps_fit = False
        
        if all_steps_fit:
            print TextFlags.AWESOME, "OK, got", len(steps), "steps with", current_configuration
            fitting_datasets.append(steps)
        
        index[configuration.keys()[0]] = index[configuration.keys()[0]] + 1
        for i in range(len(configuration)):
            parameter = configuration.keys()[i]
            
            if index[parameter] >= len(configuration[parameter]):
                if i == len(configuration) - 1:
                    run_loop = False
                    break
                else:
                    index[parameter] = 0
                    index[configuration.keys()[i + 1]] = index[configuration.keys()[i + 1]] + 1
    
    return fitting_datasets


def expandCPDs(datasets, configuration):
    cpds = []
    
    for dataset in datasets:
        cpds += expandCPD(dataset, configuration)
    
    return cpds


def evaluate_resolved_pattern(pattern, configuration):
    global loaded_datasets
    
    datasets = []
    pattern_split = pattern.split(" ")
    
    # Filter for fitting datasets
    for dataset in loaded_datasets:
        if len(dataset) > 0:
            if dataset[0]["name"] == pattern_split[0] and len(dataset[0]["call-pattern"]) == len(pattern_split) - 1:
                datasets.append(dataset)
    
    print TextFlags.INDIFFERENT, len(datasets), "structurally fitting datasets found"
    
    unbound_parameters = []
    for parameter in pattern_split[1:]:
        if not parameter in configuration:
            unbound_parameters.append(parameter)
            configuration[parameter] = []
    
    for dataset in datasets:
        step0 = dataset[0]
        
        for invocation in step0["invocations"]:
            uid_based_invocation = invocation[str(dataset[0]["uid"])]
            
            for parameter in uid_based_invocation:
                if parameter in unbound_parameters:
                    if not uid_based_invocation[parameter] in configuration[parameter]:
                        configuration[parameter].append(uid_based_invocation[parameter])
    
    print TextFlags.HAPPY, "Parameter configuration space:"
    for parameter in configuration:
        values = ""
        
        for value in configuration[parameter]:
            values = values + (" | " if not values == "" else "") + "\"" + value + "\""
        
        print TextFlags.HAPPY, "- " + parameter + " = " + values
    
    correctly_parameterized_datasets = expandCPDs(datasets, configuration)
    
    len_cpd = len(correctly_parameterized_datasets)
    print TextFlags.HAPPY, len_cpd, "dataset" + ("s" if len_cpd != 1 else ""), "fit" + ("s" if len_cpd == 1 else ""), "the bindings supplied"
    
    sys.stdout.write('[ :| ] Constructing the corresponding step-by-step plans...')
    
    returned_plans = []
    for dataset in correctly_parameterized_datasets:
        plan = Plan()
        
        for step in dataset:
            planstep = Step()
            
            planstep.type = "cram_function"
            planstep.pattern = step["name"]
            
            for parameter in step["invocations"][0]:
                bdg = Binding()
                bdg.type = 0
                bdg.key = parameter
                bdg.value = step["invocations"][0][parameter]
                
                planstep.bindings.append(bdg)
            
            plan.steps.append(planstep)
        
        returned_plans.append(plan)
    
    print "done."
    
    return returned_plans


def plan_replies(pattern, bindings):
    res = PlanningResponse()
    
    bindings = unify_bindings(bindings)
    (configurations, unused_bindings) = resolve_patterns(pattern, bindings)
    res.unused_bindings = unused_bindings
    
    for configuration in configurations:
        plans = evaluate_resolved_pattern(pattern, configuration)
        res.plans += plans
    
    if len(configurations) == 0:
        res.plans += evaluate_resolved_pattern(pattern, {})
    
    print TextFlags.SAD, "No scoring for now, sorry. Defaulting to '0.0' for all of them."
    
    print TextFlags.HAPPY, "Returning " + str(len(res.plans)) + " plan" + ("s" if len(res.plans) != 1 else "")
    
    return res


def handle_planning_request(req):
    if req.pattern != "":
        print TextFlags.INDIFFERENT, "Planning for pattern '" + req.pattern + "'"
        bindings_clean = []
        
        for binding in req.bindings:
            if binding.key != "":
                bindings_clean.append(binding)
        
        if len(bindings_clean) > 0:
            for binding in bindings_clean:
                print TextFlags.INDIFFERENT, "- " + binding.key + " = '" + binding.value + "'"
        else:
            print TextFlags.SAD, "No (non-empty) bindings defined, resolving all possible solutions."
        
        return plan_replies(req.pattern, bindings_clean)
    else:
        print TextFlags.SAD, "Empty pattern, returning zero reply (aka no plans inside)."
        return PlanningResponse()


def planning_server():
    rospy.init_node('planning_server')
    s = rospy.Service('/planning_server/plan', Planning, handle_planning_request)
    
    load_data()
    
    print TextFlags.HAPPY, "Ready to plan"
    rospy.spin()


if __name__ == "__main__":
    planning_server()
