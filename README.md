# cutplan

To try out, open three terminals. Run roslisp_repl in one and

rosrun cutplan cutplan

in the second, and run RViz in the third (I start a pr2_moveit_config/launch/demo.launch to be sure I get the /odom_combined frame published, then make the robot invisible).

In the REPL, do 

(swank:operate-on-system-for-emacs "cutplan-demo" (quote load-op))

then one of

(init-brush-test)

or 

(init-wiper-test)

This should result in some marker meshes appearing in RViz (a goal region, and in the case of the wiper test, a forbidden region-- red-- and a collector region-- green).

From this point, use either

(step-brush-test)

or

(step-wiper-test)

depending on which previous init-call you made. The goal marker will change to indicate areas that are expected to be removed by the currently proposed maneuver; also, the step-*-test function will return a list of numbers that are the parameters for the maneuver (currently, start-end positions in xyz form).

The /outputs folder in the package will be filled with goal meshes as they change under step-*-test, and this is where RViz will load them from. Delete these when no longer needed.

You should restart RViz when you want to restart a test or change the test you want to look at.


