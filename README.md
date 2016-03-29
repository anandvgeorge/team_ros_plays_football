# team_ros_plays_football
repo for Team ROS to design its robot soccer / football algorithms for the Distributed Autonomous Robotic Systems class
Romain Chiappinelli
Ryan Louie
Lukas Brunke

some changes from romain

Using the command line! Whoop Whoop!

## git help from git king ryan
	git pull
	… 
	git add –-all
	git commit -m "..."	 // git commit -am "..."
	git push

## Zone Passer Video Observation Notes

#### zonepassing3.mp4
has the full run, of passing (not perfect) and scoring! This was using r=0.06, everthing else default as of Monday March 21st.

#### zonepassing4.mp4
with parameters such as
    v2Pos(..., k=0.25)
    passPath(..., vmax=10, vr=7)
the passing becomes much more accurate, while still maintaining an acceleration at the end for maximum kicking power.  Still, it doesnt have the accuracy at the end, which leads to the kick going out into the side lines.  The precision of the kick (pass or shoot) is CRUCIAL to success.

#### zonepassing5.mp4
same as zonepassing4.mp4, with the addition
    passPath(..., ..., kq=0.005)
However, I am seeing some bugs with the path be calculated twice, for the second zone passer.  Watching video now...

#### zonepassing6.mp4
when calculating receiving destination, it seems that it is better to choose a receiving distination that is far away from the receiver (i.e. more towards center). This minimizes our risk of hitting the ball to the edges, where corner cases kills the effectiveness of our path planning/execution algorithms.
This shows successfull soft passes to all zones, minus the goal shot.

-----------------------------------------------------------
end of tuesday, march 22nd
whats next:
- path Pruning is dumb right now.  But we have serious fragility issues if the ball ever reaches the edge.  We have no smart way of getting the ball to go into a desired position if its near an object / obstacle.
- even simple kicks will not be accurate based on certain parameters.  Remember a few cases...
    1) for the first pass, if calculateReceivingDestination(..., k=0.20), it may be too much and the robot takes a weird trajectory, not hitting it towards the intended target (maybe not enough points in between
    2) for the second pass, the trajectory may look good printed out, but the execution at the end sends the ball in all sorts of directions (many times towards the edge).  Improving the accuracy in some way for consistencies sake would be SO useful.  Here's some suggestions:
    a) setting helper points in the path that make the execution more dead on
    b) changing parameters in the controller (maybe make it PI?, or make the buffer radius rb smaller, or turn down the speed once again to let it accurately achieve each point
    c) is the goal point a couple distance units in front of the ball the right strategy?  We are suffering a game of pool right now, where the robot is the cue ball.  The cue ball is not striking at either the right angle. Or it has "english" (the robot is turning at the end of it!!!!). 






### trying a smaller header

#### an even smaller header

##### is this the smallest header?



