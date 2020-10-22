
ROBOTS=2
HOUSES=2
STREETS=2
PACKAGES=4

gnome-terminal -e "roscore"
sleep 2s
gnome-terminal -e "rosrun group_16 server.py -robots $ROBOTS -houses $HOUSES -packages $PACKAGES -streets $STREETS -headless true"
sleep 2s
./planners/Metric-FF/ff -o domain.pddl -f problem.pddl | grep -i "[0-9][0-9]*:" > problem.pddl.soln
sleep 1s
rosrun group_16 execute_plan.py
rosrun group_16 test.py -command is_terminal_state