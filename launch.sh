
ROBOTS=2
HOUSES=2
STREETS=2
PACKAGES=3

gnome-terminal -e "roscore"
sleep 2s
gnome-terminal -e "rosrun group_16 server.py -robots $ROBOTS -houses $HOUSES -packages $PACKAGES -streets $STREETS"
sleep 2s
gnome-terminal -e "roslaunch group_16 neighborhood.launch"
sleep 10s
i=0
while [ $i -lt $ROBOTS ]
do
    gnome-terminal -e "rosrun group_16 move_tbot3.py -id $i"
    i=$(($i+1))
done

./planners/Metric-FF/ff -o domain.pddl -f problem.pddl | grep -i "[0-9][0-9]*:" > problem.pddl.soln
sleep 1s
rosrun group_16 execute_plan.py
rosrun group_16 test.py -command is_terminal_state