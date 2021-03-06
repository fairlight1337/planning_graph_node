#!/usr/bin/env python

#  Software License Agreement (BSD License)
#  
#  Copyright (c) 2015, Institute for Artificial Intelligence,
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
import copy
import rospy

from planning_msgs.srv import *
from planning_msgs.msg import *

from protoexp.OutputPerks import *


class ScoringMethod:
    Undefined = "undefined"
    OSPT = "optimistic shortest projected time"
    PSPT = "pessismistic shortest projected time"
    LPFO = "least projected failure occurrences"


loaded_datasets = []


def transform_invocations(node):
    invocations_fixed = []
    
    for invocation in node["invocations"]:
        invocation_fixed = {}
        for uid in invocation:
            invocation_fixed[uid] = {}
            for parameter in invocation[uid]:
                invocation_fixed[uid][param_exp_to_prolog(parameter)] = invocation[uid][parameter]
        
        invocations_fixed.append(invocation_fixed)
    
    node["invocations"] = invocations_fixed
    
    if "next-action" in node:
        transform_invocations(node["next-action"])
    
    if "child" in node:
        transform_invocations(node["child"])


def transform_callpattern(node):
    call_pattern_split = node["call-pattern"].split(" ")
    call_pattern_fixed = []
    
    if node["call-pattern"] != "":
        for parameter in call_pattern_split:
            call_pattern_fixed.append(param_exp_to_prolog(parameter))
    
    node["call-pattern"] = call_pattern_fixed
    
    if "next-action" in node:
        transform_callpattern(node["next-action"])
    
    if "child" in node:
        transform_callpattern(node["child"])


def transform_name(node):
    node["name"] = (node["name"][21:] if node["name"][:21] == "REPLACEABLE-FUNCTION-" else node["name"]).lower()
    
    if "next-action" in node:
        transform_name(node["next-action"])
    
    if "child" in node:
        transform_name(node["child"])


def process_loadable_dataset(dataset):
    # Careful: This function operates by reference, _NOT_BY_VALUE_.
    transform_invocations(dataset)
    transform_callpattern(dataset)
    transform_name(dataset)


def load_data():
    global loaded_datasets
    
    global_dir = os.path.dirname(os.path.abspath(__file__)) + "/../data"
    filenames = ["deduced_experiences.json"]
    
    if len(filenames) > 0:
        print TextFlags.MEH, "Loading plan data:"
        for filename in filenames:
            print TextFlags.HAPPY, "- " + filename
        
        with open(global_dir + "/" + filename, "r") as f:
            datasets = json.load(f)
            
            for dataset in datasets:
                process_loadable_dataset(dataset)
                loaded_datasets.append(dataset)
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


def param_exp_to_prolog(param):
    return "?" + param.lower()


def param_prolog_to_exp(param):
    return param[1:].upper()


def construct_steps(node, invocations, parent, parent_id, invocation_space, scoring_methods):
    step = Step()
    steps = []
    
    step.type = "cram_function" # Everything is a `cram_function` right now.
    step.pattern = node["name"]
    step.parent = parent
    step.id = node["node"]
    
    collected_values = {}
    for parameter in node["call-pattern"]:
        step.call_pattern.append(parameter)
        valid_invocations = get_valid_invocations(node, str(parent_id), invocation_space)
        
        for invocation in valid_invocations:
            if str(step.id) in invocation:
                parameters = invocation[str(step.id)]
                if parameter in parameters:
                    if not parameter in collected_values:
                        collected_values[parameter] = []
                    
                    if not parameters[parameter] in collected_values[parameter]:
                        collected_values[parameter].append(parameters[parameter])
    
    for parameter in collected_values:
        for value in collected_values[parameter]:
            bdg = Binding()
            bdg.type = 0 # String
            bdg.key = parameter
            bdg.value = value
            
            step.bindings.append(bdg)
    
    duration = copy.deepcopy(node["duration-confidence"])
    next_duration = [0, 0]
    failures = []
    steps_next = []
    steps_child = []
    
    if "next-action" in node:
        (steps_next, next_duration, next_failures) = construct_steps(node["next-action"], invocations, parent, parent_id, invocation_space, scoring_methods)
        failures += next_failures
    
    if "child" in node:
        (steps_child, duration, child_failures) = construct_steps(node["child"], invocations, node["node"], parent_id, invocation_space, scoring_methods)
        failures += child_failures
    
    duration[0] += next_duration[0]
    duration[1] += next_duration[1]
    
    step.score = step_score(duration, failures, scoring_methods)
    step.duration = duration
    
    steps.append(step)
    steps += steps_next
    steps += steps_child
    
    return (steps, duration, failures)


def accumulate_invocations(invocations):
    acc = {}
    
    for invocation in invocations:
        for parameter in invocation:
            if not parameter in acc:
                acc[parameter] = []
            
            if not invocation[parameter] in acc[parameter]:
                acc[parameter].append(invocation[parameter])
    
    return acc


def step_score(duration_interval, failures, scoring_methods):
    score = 1.0
    
    for scoring_method in scoring_methods:
        result = 1.0
        
        if scoring_method == ScoringMethod.OSPT: # optimistic shortest projected time
            result = 1.0 / duration_interval[0]
        elif scoring_method == ScoringMethod.PSPT: # pessimistic shortest projected time
            result = 1.0 / duration_interval[1]
        elif scoring_method == ScoringMethod.LPFO: # least projected failure occurrences
            result = 1.0 / (len(failures) + 1)
        
        score *= result
    
    return score


def construct_plan(dataset, invocations, invocation_space, scoring_methods):
    plan = Plan()
    
    (steps, duration_interval, failures) = construct_steps(dataset, invocations, -1, dataset["node"], invocation_space, scoring_methods)
    plan.steps = steps
    plan.score = step_score(duration_interval, failures, scoring_methods)
    plan.duration = duration_interval
    
    return plan


def construct_plans(datasets, configuration, scoring_methods):
    plans = []
    
    for dataset in datasets:
        valid_invocations = get_valid_invocations(dataset, str(dataset["node"]), configuration)
        
        if len(valid_invocations) > 0:
            parent_invocation_space = accumulate_invocations(valid_invocations)
            
            for parent_invocation in parent_invocation_space[str(dataset["node"])]:
                plans.append(construct_plan(dataset, valid_invocations, parent_invocation, scoring_methods))
    
    finished_plans = []
    
    for plan in plans:
        plan_ok = True
        
        for step in plan.steps:
            if len(step.bindings) != len(step.call_pattern):
                plan_ok = False
                break
        
        if plan_ok:
            finished_plans.append(plan)
    
    return finished_plans


def cmp_plan_scores(plan_a, plan_b):
    if plan_a.score > plan_b.score:
        return 1
    elif plan_a.score < plan_b.score:
        return -1
    else:
        return 0


def get_valid_invocations(dataset, parent_id, configuration):
    valid_invocations = []
    
    for invocation in dataset["invocations"]:
        if parent_id in invocation:
            valid = True
            inv_first = invocation[parent_id]
            
            for parameter in inv_first:
                if not inv_first[parameter] in configuration[parameter]:
                    valid = False
                    break
            
            if valid:
                already_present = False
                
                for v_inv in valid_invocations:
                    all_in = True
                    
                    for parameter in v_inv[parent_id]:
                        if invocation[parent_id][parameter] != v_inv[parent_id][parameter]:
                            all_in = False
                            break
                    
                    if all_in:
                        already_present = True
                        break
                
                if not already_present:
                    valid_invocations.append(invocation)
    
    return valid_invocations


def evaluate_resolved_pattern(pattern, configuration, scoring_methods):
    global loaded_datasets
    
    datasets = []
    pattern_split = pattern.split(" ")
    
    # Filter for fitting datasets
    for dataset in loaded_datasets:
        if dataset["name"] == pattern_split[0] and len(dataset["call-pattern"]) == len(pattern_split) - 1:
            datasets.append(copy.deepcopy(dataset))
    
    print TextFlags.MEH, len(datasets), "structurally fitting datasets found for '" + pattern_split[0] + "' (" + str(len(pattern_split) - 1) + " parameter" + ("s" if len(pattern_split) - 1 != 1 else "") + ")"
    
    # TODO: Remember to put in parameter transformations here for
    # allowing the caller to use differently named parameters than the
    # ones saved in the dataset! The structurally fitness test already
    # reflects this, but the machinery behind this needs to take that
    # into account as well. Should be possible by just 'recasting' the
    # configuration for every loaded dataset on the fly (as they might
    # have differently named parameters as well although the amount is
    # the same).
    
    unbound_parameters = []
    for parameter in pattern_split[1:]:
        if not parameter in configuration:
            unbound_parameters.append(parameter)
            configuration[parameter] = []
    
    for dataset in datasets:
        for invocation in dataset["invocations"]:
            uid_based_invocation = invocation[str(dataset["node"])]
            
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
    
    returned_plans = construct_plans(datasets, configuration, scoring_methods)
    
    return returned_plans


def plan_replies(pattern, bindings, scoring_methods):
    res = PlanningResponse()
    
    bindings = unify_bindings(bindings)
    (configurations, unused_bindings) = resolve_patterns(pattern, bindings)
    res.unused_bindings = unused_bindings
    
    if len(configurations) == 0:
        configurations.append({})
    
    for configuration in configurations:
        plans = evaluate_resolved_pattern(pattern, configuration, scoring_methods)
        res.plans += plans
    
    res.plans.sort(cmp_plan_scores)
    
    print TextFlags.MEH, "Metrics used for plan scoring:"
    for scoring_method in scoring_methods:
        print TextFlags.MEH, "-", scoring_method
    
    print TextFlags.HAPPY, "Returning " + str(len(res.plans)) + " plan" + ("s" if len(res.plans) != 1 else "")
    
    return res


def handle_planning_request(req):
    scoring_methods = []
    
    for method in req.scoring_methods:
        if method == "ospt":
            scoring_methods.append(ScoringMethod.OSPT)
        elif method == "pspt":
            scoring_methods.append(ScoringMethod.PSPT)
        elif method == "lpfo":
            scoring_methods.append(ScoringMethod.LPFO)
    
    if len(scoring_methods) == 0:
        print TextFlags.MEH, "No scoring method specified. Assuming 'OSPT+LPFO' as default."
        scoring_methods = [ScoringMethod.OSPT, ScoringMethod.LPFO]
    
    if req.pattern != "":
        print TextFlags.MEH, "Planning for pattern '" + req.pattern + "'"
        bindings_clean = []
        
        for binding in req.bindings:
            if binding.key != "":
                bindings_clean.append(binding)
        
        if len(bindings_clean) > 0:
            for binding in bindings_clean:
                print TextFlags.MEH, "- " + binding.key + " = '" + binding.value + "'"
        else:
            print TextFlags.SAD, "No (non-empty) bindings defined, resolving all possible solutions."
        
        return plan_replies(req.pattern, bindings_clean, scoring_methods)
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
