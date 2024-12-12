# BehaviorTree.MTC

**This project is still in an early development phase.**

This project is compatible with version 4.X of [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP).

It is recommanded that the [Moveit Task Constructor](https://github.com/moveit/moveit_task_constructor) library commit version `>=` [fdc06c3](https://github.com/moveit/moveit_task_constructor/commit/fdc06c3b9105dadec2f44ee3e4b3133986945e86). This is to ensure that one can preempt a plan gracefully.

## Demo
Check the available [examples](./examples).

Running the Pick and Place MTC demo using BT's:

``` sh
# In one terminal
$ ros2 launch moveit_task_constructor_demo demo.launch.py
# ALTERNATE: Use the one supplied with MTC
#            Don't forget to change the task solution topic

# In another terminal
$ ros2 launch behaviortree_mtc run.launch.py


# OPTIONAL: Connect from Groot2 to visualize the BT's on port 1667
$ ros2 launch behaviortree_mtc run.launch.py delay_ms:=5000
```

## Roadmap
We plan to release version `1.0.0` by the end of Christmas 2024. Check the [roadmap](https://github.com/captain-yoshi/BehaviorTree.MTC/issues/15) for more details.
