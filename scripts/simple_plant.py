#!/usr/bin/env python3
import os
import sys

current_path = os.path.dirname(os.path.abspath(__file__))
parent_path = os.path.abspath(os.path.join(current_path, ".."))
sys.path.insert(0, parent_path)

import rospy
from std_msgs.msg import String
from ultrades.automata import state, event, dfa
from core.utils import load_from_json
from core.AutomatonNode import AutomatonNode 

def create_simple_automaton():
    q0 = state("q0", marked=True)
    q1 = state("q1")

    a = event("a", controllable=True)
    b = event("b", controllable=False)

    transitions = [
        (q0, a, q1),
        (q1, b, q0)
    ]

    return dfa(transitions, q0, "SimpleDFA")

if __name__ == "__main__":
    automaton = create_simple_automaton()
    node = AutomatonNode("simple_test_node", dfa_obj=automaton, role="plant")
    node.run()
