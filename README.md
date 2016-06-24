# cutplan

You also need to have the meshproc packages (meshproc_msgs and meshproc_csg) installed.

To try out, open three terminals. Run roslisp_repl in one and

rosrun meshproc_csg meshproc_csg

in the second, and run RViz in the third (I start a pr2_moveit_config/launch/demo.launch to be sure I get the /odom_combined frame published, then make the robot invisible).

In the REPL, do 

(swank:operate-on-system-for-emacs "cutplan" (quote load-op))

(cutplan:start-ros-node)

(cut-test)

This should result in some marker meshes appearing in RViz (an object-- a "pizza"-- with a slice cut out). You will also see a vector of doubles being the return value of the cut-test function. These are the parameters for the cut. In this vector, three consequent numbers (indices 0-2, 3-5, 6-8 etc.) are xyz coordinates of a point. The points are listed as start, end of a segment.

The ./outputs folder contains temporary files that are meshes for the RViz markers. Delete these when no longer needed.

You should restart RViz when you want to restart a test or change the test you want to look at.


