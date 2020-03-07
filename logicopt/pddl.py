#!/usr/bin/env python

from logicopt import Validator

if __name__ == '__main__':
    val = Validator("../resources/domain.pddl", "../resources/problem.pddl")
    print("Initial state:", val.initial_state)

    assert val.is_valid_action(val.initial_state, "pick(hook)")

    assert not val.is_valid_action(val.initial_state, "pick(table)")

    next_state = val.next_state(val.initial_state, "pick(hook)")
    print("pick(hook) -> ", next_state)

    assert val.is_valid_tuple(val.initial_state, "pick(hook)", next_state)
    
    false_state = set(val.initial_state)
    false_state.add("inhand(hook)")
    assert not val.is_valid_tuple(val.initial_state, "pick(hook)", false_state)

    goal_state = set(val.initial_state)
    goal_state.remove("on(box, table)")
    goal_state.add("on(box, shelf)")
    assert val.is_goal_satisfied(goal_state)

    assert not val.is_goal_satisfied(val.initial_state)

    assert val.is_valid_plan(["pick(hook)",
                              "push(hook,box,table)",
                              "place(hook,table)",
                              "pick(box)",
                              "place(box,shelf)"])

    assert not val.is_valid_plan(["pick(hook)",
                                  "push(hook,box,table)",
                                  "place(hook,table)",
                                  "pick(box)",
                                  "place(box,hook)"])
