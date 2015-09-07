#!/usr/bin/python

import sys
import json
import copy
import math

from OwlReader import OwlReader
from DesignatorReader import DesignatorReader
from Log import Log


class MemoryCondenser:
    def __init__(self, parent=None):
        self.rdrOwl = OwlReader()
        self.rdrDesig = DesignatorReader()
        
        self.arrExperiences = []
        
        self.knownTaskTypes = []
    
    def countExperiences(self):
        return len(self.arrExperiences)
    
    def addExperience(self, expAdd):
        self.arrExperiences.append(expAdd)
    
    def loadExperience(self, strOwlFile, strDesignatorFile):
        logReturn = Log()
        
        logReturn.setOwlData(self.rdrOwl.loadOwl(strOwlFile))
        
        if strDesignatorFile != "":
            logReturn.setDesignatorData(self.rdrDesig.loadDesignators(strDesignatorFile))
        
        self.addExperience(logReturn)
    
    def durations_for_task_type(self, data, task_type):
        collected_durations = []
        
        for key in data:
            if key == task_type:
                collected_durations += data[key]["durations"]
            
            collected_durations += self.durations_for_task_type(data[key]["children"], task_type)
        
        return collected_durations
    
    def condenseData(self, dataOwl):
        result = None
        
        self.tti = dataOwl["task-tree-individuals"]
        owlMeta = dataOwl["metadata"]
        owlAnnot = dataOwl["annotation"]
        
        if owlMeta:
            result = {"Toplevel" : self.condenseNodes("", owlMeta.subActions())};
        else:
            print "No meta data in file!"
        
        return result
    
    def condenseNodes(self, strParentNode, arrNodes, nLevel = 0):
        arrTypes = {}
        arrIndividuals = {}
        
        for strNode in arrNodes:
            owlNode = self.tti[strNode]
            ident = owlNode.taskContext()
            
            failures = owlNode.failures()
            failure = ""
            if len(failures) > 0:
                failure = self.tti[failures[0]].type()
            
            result = self.condenseNodes(strNode, owlNode.subActions(), nLevel + 1)
            if not ident in arrTypes:
                arrTypes[ident] = result
            else:
                arrTypes[ident] = self.unifyResults(arrTypes[ident], result)
            
            arrTypes[ident]["individuals"][strNode] = {"parameters" : owlNode.annotatedParameters(True),
                                                       "parent" : strParentNode,
                                                       "failure" : failure}
        
        return {"subTypes" : arrTypes,
                "individuals" : {}}
    
    def unifyResults(self, res1, res2):
        resparams = {}
        if len(res1["individuals"]) > 0:
            resparams = res1["individuals"]
        
        if len(res2["individuals"]) > 0:
            resparams = dict(resparams.items() + res2["individuals"].items())
        
        unified = {"subTypes" : {},
                   "individuals" : resparams}
        
        for ressub1 in res1["subTypes"]:
            if ressub1 in res2["subTypes"]:
                unified["subTypes"][ressub1] = self.unifyResults(res1["subTypes"][ressub1],
                                                                 res2["subTypes"][ressub1])
            else:
                unified["subTypes"][ressub1] = res1["subTypes"][ressub1]
        
        for ressub2 in res2["subTypes"]:
            if not ressub2 in res1["subTypes"]:
                unified["subTypes"][ressub2] = res2["subTypes"][ressub2]
        
        return unified
    
    def condense(self):
        arrStartNodes = []
        
        self.tti = {}
        
        for experience in self.arrExperiences:
            owlData = experience.getOwlData()
            metaData = owlData["metadata"]
            arrStartNodes += metaData.subActions()
            
            self.tti.update(owlData["task-tree-individuals"])
        
        self.processed_nodes = []
        tree = self.condenseNodesByContext(arrStartNodes)
        parameters = {}
        
        for node in self.processed_nodes:
            params = self.tti[node].annotatedParameters(bSingularParameters = True)
            
            if len(params) > 0:
                parameters[node] = {}
                for p in params:
                    if not p == "_time_created":
                        parameters[node][p.lstrip("parameter-")] = params[p]
        
        result = {"tree": tree,
                  "parameters": parameters}
        
        print result
    
    def sortComparatorActionTime(self, action1, action2):
        if action1.timeSpan[0] > action2.timeSpan[0]:
            return 1
        elif action1.timeSpan[0] == action2.timeSpan[0]:
            return 0
        else:
            return -1
    
    def sortActionsByTime(self, actions):
        actions.sort(self.sortComparatorActionTime)
        
        return actions
    
    def condenseNodesByContext(self, nodes):
        # Separate nodes by their taskContext
        dicContexts = {}
        
        for node in nodes:
            self.processed_nodes.append(node)
            
            owlNode = self.tti[node]
            
            if not owlNode.taskContext() in dicContexts:
                dicContexts[owlNode.taskContext()] = {"nodes": [],
                                                      "terminal-state": []}
            
            dicContexts[owlNode.taskContext()]["nodes"].append(node)
        
        for context in dicContexts:
            all_children = []
            
            for node in dicContexts[context]["nodes"]:
                sub_actions = self.sortActionsByTime(self.tti[node].subActions())
                
                if len(sub_actions) > 0:
                    all_children += sub_actions
                else:
                    dicContexts[context]["terminal-state"].append(node)
            
            dicContexts[context]["children"] = self.condenseNodesByContext(all_children)
        
        return dicContexts
    
    def generalizeExperiences(self):
        self.generalizedExperience = {}
        self.tti = {}
        
        arrStartNodes = []
        
        for experience in self.arrExperiences:
            owlData = experience.getOwlData()
            metaData = owlData["metadata"]
            arrStartNodes += metaData.subActions()
            
            self.tti.update(owlData["task-tree-individuals"])
        
        for node in arrStartNodes:
            self.injectActionIntoGeneralizedExperience(node, self.generalizedExperience)
        
        print self.generalizedExperience
    
    def injectActionIntoGeneralizedExperience(self, action, target_branch):
        target_branch["a"] = 5
    
    def dotNode(self, node, first):
        dot = ""
        tti = self.t_tti[node]
        former_subnode = ""
        
        dot += "  {rank=same;"
        for subnode in tti.subActions():
            dot += " " +subnode
        dot += "}\n"
        
        for subnode in tti.subActions():
            dot += "  " + subnode + " [shape=box, label=\"" + self.t_tti[subnode].taskContext() + "\"]\n"
            #dot += "  edge ";
            
            if first == True:
                first = False
                dot += "edge [dir=both, arrowhead=normal, arrowtail=none]"
                dot += "\n  " + node + " -> " + subnode + "\n"
            else:
                pass#dot += "[dir=both, arrowhead=diamond, arrowtail=ediamond]"
            
            
            if not former_subnode == "":
                dot += "  edge [arrowhead=empty, arrowtail=none]\n"
                dot += "  " + former_subnode + " -> " + subnode + "\n"
            
            dot += self.dotNode(subnode, True)
            
            former_subnode = subnode
        
        if len(tti.subActions()) == 0 and tti.nextAction() == None:
            #dot += "  terminal_state_" + node + " [shape=doublecircle, label=\"\"]\n"
            #dot += "  edge [arrowhead=empty, arrowtail=none]\n"
            #dot += "  " + node + " -> terminal_state_" + node + "\n"
            pass
        
        return dot
    
    def printExperiences(self, dot):
        for experience in self.arrExperiences:
            if dot:
                self.printDotExperience(experience)
            else:
                self.printRawExperience(experience)
    
    def printRawExperience(self, experience):
        owlData = experience.getOwlData()
        metaData = owlData["metadata"]
        start_nodes = metaData.subActions()
        self.t_tti = owlData["task-tree-individuals"]
        
        for node in start_nodes:
            self.printRawExperienceNode(node)
    
    def printRawExperienceNode(self, node, level = 0):
        indent = "   " * level
        owl = self.t_tti[node]
        
        parameters = owl.annotatedParameters()
        param_str = "("
        first = True
        
        for parameter in parameters:
            if not parameter == "_time_created":
                if first == True:
                    first = False
                else:
                    param_str = param_str + ", "
                
                key_str = parameter[10:] if parameter[:10] == "parameter-" else parameter
                param_str = param_str + key_str + "=" + parameters[parameter][0]
        
        param_str = param_str + ")"
        
        print indent + owl.taskContext() + " " + param_str
        
        if len(owl.subActions()) > 0:
            for node in owl.subActions():
                self.printRawExperienceNode(node, level + 1)
    
    def printDotExperience(self, experience):
        owlData = experience.getOwlData()
        metaData = owlData["metadata"]
        start_nodes = metaData.subActions()
        self.t_tti = owlData["task-tree-individuals"]
        
        dot = "digraph plangraph {\n"
        dot += "  label=\"Original Experiences\"\n"
        dot += "  labeljust=center\n"
        dot += "  labelloc=top\n"
        
        for node in start_nodes:
            dot += "  " + node + " [shape=box, label=\"" + self.t_tti[node].taskContext() + "\"]\n\n"
            
            dot += self.dotNode(node, True)
        
        dot += "}\n"
        
        print dot
    
    def compareSubActions(self, subaction1, subaction2):
        if subaction1 == subaction2:
            return 0
        
        next_action = subaction1
        
        while next_action != None:
            next_action = self.tti[subaction1].nextAction()
            
            if next_action == subaction2:
                return 1
        
        return -1
    
    def sortSubActions(self, subactions):
        subactions.sort(self.compareSubActions)
        
        return subactions
    
    def sortSubActionsList(self, subactions_list):
        sorted_list = []
        
        for subactions in subactions_list:
            sorted_list.append(self.sortSubActions(subactions))
        
        return sorted_list
    
    def generalizeNodes(self, nodes):
        sequences = []
        
        for node in nodes:
            sequences.append(self.tti[node].subActions())
        
        return sequences
    
    def workOnExperiences(self):
        start_nodes = []
        self.tti = {}
        
        for experience in self.arrExperiences:
            owlData = experience.getOwlData()
            metaData = owlData["metadata"]
            
            start_nodes += metaData.subActions()
            self.tti.update(owlData["task-tree-individuals"])
        
        print self.generalizeNodes(start_nodes)
    
    def injectExperiences(self, deduced = False, data = False):
        self.arrInjected = {}
        self.tti = {}
        self.uid_counter = 0;
        
        root_action_count = 0
        
        for experience in self.arrExperiences:
            owlData = experience.getOwlData()
            metaData = owlData["metadata"]
            self.tti.update(owlData["task-tree-individuals"])
            
            for node in metaData.subActions():
                self.injectExperienceNode(node, self.arrInjected, True)
                root_action_count = root_action_count + 1
        
        for experience in self.arrExperiences:
            owlData = experience.getOwlData()
            metaData = owlData["metadata"]
            self.tti.update(owlData["task-tree-individuals"])
            
            for node in metaData.subActions():
                self.checkForTerminalStateOccurrences(self.tti[node].taskContext(), self.arrInjected)
            
            for node in metaData.subActions():
                self.checkForOptionalInjectedNodes(self.tti[node].taskContext(), self.arrInjected)
        
        self.taskDurations = {}
        for task_type in self.knownTaskTypes:
            self.taskDurations[task_type] = self.durations_for_task_type(self.arrInjected, task_type)
        
        if deduced:
            self.printDeduced(dot = not data, root_action_count = root_action_count)
        else:
            self.printInjected(dot = not data)
    
    def getParameters(self, node):
        params_fixed = {}
        call_pattern = ""
        
        params = self.tti[node].annotatedParameters()
        
        for param in params:
            if not param == "_time_created" and not param == "CALLPATTERN":
                key_str = param[10:] if param[:10] == "parameter-" else param
                params_fixed[key_str] = params[param][0]
            elif param == "CALLPATTERN":
                call_pattern = params[param][0]
        
        return (call_pattern, params_fixed)
    
    def emptyContext(self, invocation_path, call_pattern):
        ctx = {"children": {},
               "next-actions" : {},
               "uid" : self.uid_counter,
               "terminal-state": "false",
               "start-state": "false",
               "optional": "false",
               "instances": 0,
               "durations": [],
               "invocations": [invocation_path],
               "call-pattern": call_pattern}
        self.uid_counter = self.uid_counter + 1
        
        return ctx
    
    def updateFrameContext(self, node, frame, invocation_path, previous_ctx):
        ctx = self.tti[node].taskContext()
        
        if ctx != previous_ctx:
            (call_pattern, params) = self.getParameters(node)
            
            if not ctx in frame:
                invocation_path.update({self.uid_counter: params})
                frame[ctx] = self.emptyContext(invocation_path, call_pattern)
            else:
                invocation_path.update({frame[ctx]["uid"]: params})
                frame[ctx]["invocations"].append(invocation_path)
            
            if frame[ctx]["start-state"] == "false":
                if not self.tti[node].previousAction():
                    frame[ctx]["start-state"] = "true"
            
            frame[ctx]["instances"] += 1
            frame[ctx]["durations"].append(self.tti[node].time())
            
            if previous_ctx:
                if not ctx in frame[previous_ctx]["next-actions"]:
                    frame[previous_ctx]["next-actions"][ctx] = []
                
                frame[previous_ctx]["next-actions"][ctx].append(params)
    
    def injectExperienceNode(self, node, frame, rootlevel = False, invocation_path = {}, previous_node = None):
        ctx = self.tti[node].taskContext()
        new_invocation_path = invocation_path.copy()
        
        if not ctx in self.knownTaskTypes:
            self.knownTaskTypes.append(ctx)
        
        self.updateFrameContext(node, frame, new_invocation_path, self.tti[previous_node].taskContext() if previous_node else None)
        num_children = len(frame[ctx]["children"])
        num_siblings = len(frame[ctx]["next-actions"])
        
        for sub in self.tti[node].subActions():
            if not self.tti[sub].previousAction():
                self.injectExperienceNode(sub, frame[ctx]["children"], False, new_invocation_path)
        
        next_node = self.tti[node].nextAction()
        
        if next_node and not rootlevel and not ctx == self.tti[next_node].taskContext():
            self.injectExperienceNode(next_node, frame, False, new_invocation_path, node)
    
    def checkForOptionalInjectedNodes(self, ctx, frame, parent_instances = -1, came_from = None):
        if not "check-optional" in frame[ctx]:
            frame[ctx]["check-optional"] = "done"
            frame[ctx]["optional"] = "false"
            
            came_from_terminates = False
            came_from_valid = True
            if came_from:
                if came_from == ctx:
                    came_from_valid = False
                
                if frame[came_from]["terminal-state"] == "true":
                    came_from_terminates = True
            
            if came_from_valid == True:
                if frame[ctx]["instances"] < parent_instances:
                    frame[ctx]["optional"] = "true"
            
            for child in frame[ctx]["children"]:
                if frame[ctx]["children"][child]["start-state"] == "true":
                    self.checkForOptionalInjectedNodes(child, frame[ctx]["children"], frame[ctx]["instances"])
            
            for next_action in frame[ctx]["next-actions"]:
                if next_action in frame and not next_action == ctx:
                    self.checkForOptionalInjectedNodes(next_action, frame, frame[ctx]["instances"], ctx)
    
    def checkForTerminalStateOccurrences(self, ctx, frame):
        if not "check-terminal" in frame[ctx]:
            frame[ctx]["check-terminal"] = "done"
            frame[ctx]["terminal-state"] = "true"
            
            child_instances = 0
            next_instances = 0
            
            for child in frame[ctx]["children"]:
                if frame[ctx]["children"][child]["start-state"] == "true":
                    child_instances = child_instances + frame[ctx]["children"][child]["instances"]
            
            for next_action in frame[ctx]["next-actions"]:
                if next_action in frame and not next_action == ctx:
                    next_instances = next_instances + frame[next_action]["instances"]
            
            terminal_instances = frame[ctx]["instances"] - (child_instances + next_instances)
            
            if terminal_instances > 0:
                frame[ctx]["terminal-instances"] = terminal_instances
            else:
                frame[ctx]["terminal-instances"] = 0
            
            for child in frame[ctx]["children"]:
                self.checkForTerminalStateOccurrences(child, frame[ctx]["children"])
            
            for next_action in frame[ctx]["next-actions"]:
                if next_action in frame and not next_action == ctx:
                    self.checkForTerminalStateOccurrences(next_action, frame)
    
    def printDeduced(self, dot = False, root_action_count = 1):
        # TODO: Extend this to use all top-level nodes in case they
        # are different
        self.global_ctx_counter = 0
        deduced = self.expandPathways(self.arrInjected.keys()[0], self.arrInjected, root_action_count)
        
        fixed_deduced = []
        for d in deduced:
            # Go down two levels
            fixed_singular = d["child"]["child"]
            
            for invocation in fixed_singular["invocations"]:
                invocation.pop(0, 0)
                invocation.pop(1, 0)
            
            fixed_deduced.append(fixed_singular)
        
        with open("deduced_experiences.json", "w") as f:
            json.dump(fixed_deduced, f)
        
        if dot:
            self.printDotDeduced(deduced)
        else:
            print deduced
    
    def allPossibleCombinations(self, n, static_indices):
        solutions = []
        prototype = []
        
        for i in range(n):
            prototype.append(1 if i in static_indices else 0)
        
        run = True
        while run:
            prototype[0] = prototype[0] + 1
            
            for i in range(n):
                if prototype[i] > 1:
                    prototype[i] = 1 if i in static_indices else 0
                    
                    if i < n - 1:
                        prototype[i + 1] = prototype[i + 1] + 1
                    else:
                        run = False
            
            solutions.append(copy.deepcopy(prototype))
        
        return solutions
    
    def allPossiblePermutationIndices(self, lengths):
        indices = []
        for i in range(len(lengths)):
            indices.append(0)
        
        solutions = []
        
        if len(indices) > 0:
            run = True
            while run:
                indices[0] = indices[0] + 1
                
                for i in range(len(lengths)):
                    if indices[i] >= lengths[i]:
                        indices[i] = 0
                        
                        if i == len(lengths) - 1:
                            run = False
                        else:
                            indices[i + 1] = indices[i + 1] + 1
                
                solutions.append(copy.deepcopy(indices))
        
        return solutions
    
    def allPossiblePermutations(self, objects):
        lengths = []
        for object in objects:
            lengths.append(len(object))
        
        indices = self.allPossiblePermutationIndices(lengths)
        solutions = []
        
        for index in indices:
            solution = []
            
            for i in range(len(objects)):
                solution.append(objects[i][index[i]])
            
            solutions.append(copy.deepcopy(solution))
        
        return solutions
    
    def allPossibleSingularPermutations(self, objects, static_indices):
        objects_extended = []
        for i in range(len(objects)):
            add = [object, None] if not i in static_indices else [object]
            objects_extended.append(add)
        
        solutions_pre = self.allPossiblePermutations(objects_extended)
        solutions = []
        
        for solution_pre in solutions_pre:
            for bit in solution_pre:
                if bit != None:
                    solutions.append(bit)
        
        return solutions
    
    def expandPaths(self, ctx, nodes, trace):
        paths = []
        
        node = nodes[ctx]
        if not node["uid"] in trace:
            ci_n = len(node["durations"])
            ci_N = len(self.taskDurations[ctx])
            
            # https://www.stat.tamu.edu/~lzhou/stat302/standardnormaltable.pdf
            ci_alpha = 0.05 # Not really used, more for reference
            ci_z = 1.96 # From standard distribution table for alpha = 0.5 (z_{1-alpha/2})
            
            ci_x_bar = sum(node["durations"]) / float(ci_n)
            ci_s_square = 0
            for i in range(ci_n):
                ci_s_square += (node["durations"][i] - ci_x_bar) * (node["durations"][i] - ci_x_bar)
            
            ci_s_square /= (ci_n - 1)
            ci_sigma_hat_x_bar_pre = (ci_s_square / ci_n) * (1 - ci_n / ci_N)
            
            if abs(ci_sigma_hat_x_bar_pre) > 0:
                ci_sigma_hat_x_bar = math.sqrt(ci_sigma_hat_x_bar_pre)
            else:
                ci_sigma_hat_x_bar = 0
            
            ci = [ci_x_bar - ci_z * ci_sigma_hat_x_bar, ci_x_bar + ci_z * ci_sigma_hat_x_bar]
            
            node_desc = {"node": node["uid"],
                         "name": ctx,
                         "durations": node["durations"],
                         "optional": node["optional"],
                         "duration-confidence": ci,
                         "invocations": [],
                         "instances": node["instances"],
                         "call-pattern": node["call-pattern"],
                         "theoretical": "false"}
            current_trace = copy.deepcopy(trace) + [node_desc["node"]]
            
            for invocation in node["invocations"]:
                invocation_fits = True
                
                for uid in invocation:
                    if not uid in current_trace:
                        invocation_fits = False
                        break
                
                if invocation_fits:
                    node_desc["invocations"].append(invocation)
            
            if len(node_desc["invocations"]) == 0:
                node_desc["theoretical"] = "true"
            
            child_paths = []
            
            for child in self.getStartNodes(node["children"]):
                expanded_child_paths = self.expandPaths(child, node["children"], current_trace)
                child_paths += expanded_child_paths
            
            next_action_paths = []
            
            for next_action in node["next-actions"]:
                if next_action != ctx:
                    expanded_next_action_paths = self.expandPaths(next_action, nodes, current_trace)
                    next_action_paths += expanded_next_action_paths
            
            if len(child_paths) > 0 and len(next_action_paths) > 0:
                children_optional = True
                
                for child_path in child_paths:
                    current_child_copy = copy.deepcopy(node_desc)
                    current_child_copy["child"] = child_path
                    
                    if child_path["optional"] != "true":
                        children_optional = False
                    
                    next_actions_optional = True
                    
                    for next_action in next_action_paths:
                        current_child_next_copy = copy.deepcopy(current_child_copy)
                        current_child_next_copy["next-action"] = next_action
                        
                        if next_action["optional"] != "true":
                            next_actions_optional = False
                        
                        paths.append(current_child_next_copy)
                    
                    if next_actions_optional:
                        paths.append(current_child_copy)
                if children_optional:
                    paths.append(node_desc)
            elif len(child_paths) > 0:
                children_optional = True
                
                for child_path in child_paths:
                    current_child_copy = copy.deepcopy(node_desc)
                    current_child_copy["child"] = child_path
                    
                    if child_path["optional"] != "true":
                        children_optional = False
                    
                    paths.append(current_child_copy)
                
                if children_optional:
                    paths.append(node_desc)
            elif len(next_action_paths) > 0:
                next_actions_optional = True
                
                for next_action in next_action_paths:
                    current_next_copy = copy.deepcopy(node_desc)
                    current_next_copy["next-action"] = next_action
                    
                    if next_action["optional"] != "true":
                        next_actions_optional = False
                    
                    paths.append(current_next_copy)
                
                if next_actions_optional:
                    paths.append(node_desc)
            else:
                paths.append(node_desc)
        
        return paths
    
    def printPath(self, path, indentation = "", is_next = False):
        sys.stdout.write(indentation + str(path["node"]))
        
        if "next-action" in path:
            self.printPath(path["next-action"], "   ", True)
        
        if not is_next:
            sys.stdout.write("\n")
        
        if "child" in path:
            if is_next:
                sys.stdout.write("   \n")
            
            self.printPath(path["child"], "    " if is_next else "")
    
    def expandPathways(self, ctx, nodes, root_action_count, trace = [], relation = "root", correspondant = ""):
        pathways = self.expandPaths(ctx, nodes, [])
        
        return pathways
    
    def getStartNodes(self, nodes):
        start_nodes = {}
        
        for node in nodes:
            if nodes[node]["start-state"] == "true":
                start_nodes[node] = nodes[node]
        
        return start_nodes
    
    def printInjected(self, dot = False):
        if dot:
            self.printInjectedDot()
        else:
            print self.arrInjected
    
    def printInjectedChildren(self, children, parent = None):
        dot = ""
        edge_pointers = {}
        next_action_parameters = {}
        ids = {}
        optionals = {}
        
        parent_id = parent
        if not parent:
            parent_id = "root"
        
        for child in children:
            child_id = "node_" + child.replace("-", "_") + "_" + str(self.counterdot)
            ids[child] = child_id
            
            self.counterdot = self.counterdot + 1
            
            label = child
            if label[:21] == "REPLACEABLE-FUNCTION-":
                label = label[21:]
            
            call_pattern = children[child]["call-pattern"]
            
            dot += "  " + child_id + " [shape=box, label=\"" + label + " (" + str(children[child]["uid"]) + " / " + str(children[child]["instances"]) + ")\n" + call_pattern + "\"]\n"
            
            if children[child]["terminal-state"] == "true":
                if children[child]["terminal-instances"] > 0:
                    dot += "    ts_" + str(self.counterdot) + " [shape=doublecircle, label=\"" + str(children[child]["terminal-instances"]) + "\"]\n"
                    dot += "    edge [style=dashed, arrowhead=normal, arrowtail=none, label=\"terminal\"]\n"
                    dot += "    " + child_id + " -> " + "ts_" + str(self.counterdot) + "\n"
            
            dot += self.printInjectedChildren(children[child]["children"], child_id)
            
            if parent:
                if children[child]["start-state"] == "true":
                    if children[child]["optional"] == "true":
                        dot += "  edge [style=solid, arrowhead=normal, arrowtail=none, label=\"optional\"]\n"
                    else:
                        dot += "  edge [style=solid, arrowhead=normal, arrowtail=none, label=\"\"]\n"
                else:
                    if children[child]["optional"] == "true":
                        dot += "  edge [style=dashed, arrowhead=none, arrowtail=none, label=\"\"]\n"
                    else:
                        dot += "  edge [style=dashed, arrowhead=none, arrowtail=none, label=\"\"]\n"
                
                dot += "  " + parent + " -> " + child_id + "\n"
            
            for na in children[child]["next-actions"]:
                if parent:
                    if not na in edge_pointers:
                        edge_pointers[na] = []
                        next_action_parameters[na] = {}
                    
                    if not child_id in next_action_parameters[na]:
                        next_action_parameters[na][child_id] = []
                    
                    edge_pointers[na].append(child_id)
        
        for child in children:
            child_id = ids[child]
            
            if child in edge_pointers:
                for target in edge_pointers[child]:
                    param_str = ""
                    # for param_sets in next_action_parameters[child][target]:
                    #     for param_set in param_sets:
                    #         first_p = True
                    #         for p in param_set:
                    #             if first_p:
                    #                 first_p = False
                    #             else:
                    #                 param_str = param_str + ", "
                                
                    #             param_str = param_str + p + " = " + param_set[p]
                            
                    #         param_str = param_str + "\\n"
                    #if next_action_parameters[
                    
                    if children[child]["optional"] == "true":
                        param_str = "optional"
                    else:
                        param_str = ""
                    
                    dot += "  {rank=same; " + child_id + " " + target + "}\n"
                    dot += "  edge [style=solid, arrowhead=empty, arrowtail=none, label=\"" + param_str + "\"]\n"
                    dot += "  " + target + " -> " + child_id + "\n"
        
        return dot
    
    def printInjectedDot(self):
        self.counterdot = 0
        self.edge_pointers = {}
        
        dot = "digraph condensed {\n"
        dot += "  graph []\n"#ranksep=0.5#nodesep=0.5#pad=0.5
        dot += "  label=\"Condensed Experience Graph\"\n"
        dot += "  labeljust=center\n"
        dot += "  labelloc=top\n"
        dot += self.printInjectedChildren(self.arrInjected)
        dot += "}\n"
        
        print dot
    
    def expScore(self, exp):
        acc_score = 1.0
        
        for item in exp:
            instances = item["instances"]
            rel_occ = item["rel-occ"]
            acc_score = acc_score * rel_occ
        
        last_item = exp[len(exp) - 1]
        acc_score = acc_score * last_item["rel-term"]
        
        return acc_score
    
    def expScoreCmp(self, exp1, exp2):
        score1 = self.expScore(exp1)
        score2 = self.expScore(exp2)
        
        if score1 < score2:
            return 1
        elif score1 > score2:
            return -1
        else:
            return 0
    
    def reconstructItem(self, root, sequential):
        if not "children" in root:
            root["children"] = []
        
        if not "next" in root:
            root["next"] = None
        
        for item in sequential:
            #print item["correspondant"], root["uid"]
            if item["correspondant"] == root["uid"]:
                enriched_item = self.reconstructItem(item, sequential)
                
                if item["relation"] == "child":
                    root["children"].append(enriched_item)
                elif item["relation"] == "sibling":
                    root["next"] = enriched_item
        
        return root
    
    def reassambleStructure(self, sequential):
        root = None
        
        for item in sequential:
            #sys.stderr.write(str(item))
            if item["relation"] == "root":
                root = item
                break
        
        if root:
            root = self.reconstructItem(root, sequential)
        
        return root
    
    def printDeducedPlan(self, plan, indentation):
        dot = ""
        
        this_node = "node_" + str(self.node_counter)
        name = (plan["name"][21:] if plan["name"][:21] == "REPLACEABLE-FUNCTION-" else plan["name"]).lower()
        
        line_name = name + " (ID " + str(plan["node"]) + "" + (", theoretical" if plan["theoretical"] == "true" else "") + ")"
        line_call_pattern = plan["call-pattern"]
        line_invocations = str(len(plan["invocations"])) + " invocation" + ("" if len(plan["invocations"]) == 1 else "s")
        
        line_name = line_name + ("" if line_name == "" else "\n")
        line_call_pattern = line_call_pattern + ("" if line_call_pattern == "" else "\n")
        line_invocations = line_invocations + ("" if line_invocations == "" else "\n")
        
        dot += indentation + this_node + " [shape=box, label=\"" + line_name + line_call_pattern + line_invocations + "\"];\n"
        self.node_counter = self.node_counter + 1
        
        if "next-action" in plan:
            (dot_new, that_node) = self.printDeducedPlan(plan["next-action"], indentation)
            dot += dot_new
            
            dot += indentation + "{rank=same; " + this_node + " " + that_node + "};\n"
            
            dot += indentation + "edge [arrowhead=empty, arrowtail=none, label=\"sequence\"]\n"
            dot += indentation + this_node + " -> " + that_node + ";\n"
        
        if "child" in plan:
            (dot_new, that_node) = self.printDeducedPlan(plan["child"], indentation)
            dot += dot_new
            
            dot += indentation + "edge [arrowhead=normal, arrowtail=none, label=\"child\"]\n"
            dot += indentation + this_node + " -> " + that_node + ";\n"
        
        return (dot, this_node)
    
    def printDotDeduced(self, deduced):
        self.node_counter = 0
        
        counter = 0
        subgraphcounter = 0
        #print len(deduced)
        #print deduced
        #exit(-1)
        dot = "digraph deduced {\n"
        dot += "  label=\"Deduced Possible Action Paths\"\n"
        dot += "  labeljust=center\n"
        dot += "  labelloc=top\n"
        
        cluster_index = 0
        for plan in deduced:
            dot += "  \n"
            dot += "  subgraph cluster_" + str(cluster_index) + " {\n"
            dot += "    label=\"\";\n"
            (cluster, something) = self.printDeducedPlan(plan, "    ");
            dot += cluster
            dot += "  }\n"
            
            cluster_index = cluster_index + 1
        
        dot += "}\n"
        
        print dot
        
        
        
        # highest_score = 0
        # for line in deduced:
        #     acc_score = self.expScore(line)
            
        #     if acc_score > highest_score:
        #         highest_score = acc_score
        
        # deduced.sort(self.expScoreCmp)
        # deduced_reassambled = []
        # for d in deduced:
        #     deduced_reassambled.append(self.reassambleStructure(d))
        # #print deduced_reassambled
        
        # for line in deduced:
        #     dot += "  \n"
            
        #     dot += "  subgraph cluster_" + str(subgraphcounter) + " {\n"
        #     dot += "    pencolor=transparent;\n"
        #     dot += "    \n"
        #     subgraphcounter = subgraphcounter + 1
            
        #     first = True
        #     acc_score = 1.0
            
        #     for item in line:
        #         instances = item["instances"]
        #         node = item["node"]
        #         rel_occ = item["rel-occ"]
                
        #         # Correct node label
        #         if node[:21] == "REPLACEABLE-FUNCTION-":
        #             node = node[21:]
                
        #         acc_score = acc_score * rel_occ
                
        #         if not first:
        #             dot += "    node_" + str(counter - 1) + " -> node_" + str(counter) + "\n"
        #         else:
        #             first = False
                
        #         dot += "    node_" + str(counter) + " [shape=box, label=\"" + node + " (" + str(round(rel_occ, 2)) + ", " + item["relation"] + ")\"]\n"
        #         counter = counter + 1
            
        #     last_item = line[len(line) - 1]
            
        #     dot += "    ts_" + str(counter - 1) + " [shape=doublecircle, label=\"" + str(round(last_item["rel-term"], 2)) + "\"]\n"
        #     dot += "    edge [style=dashed, arrowhead=normal, arrowtail=none, label=\"\"]\n"
        #     dot += "    node_" + str(counter - 1) + " -> " + "ts_" + str(counter - 1) + "\n"
            
        #     acc_score = acc_score * last_item["rel-term"]
            
        #     dot += "    \n"
        #     dot += "    label=\"Score: " + str(round(acc_score, 2)) + "\";\n"
        #     dot += "    labeljust=center;\n"
        #     dot += "    labelloc=top;\n"
        #     dot += "  }\n"
            
        # dot += "}\n"
        
        # print dot
