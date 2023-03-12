1.Project Name: 	GuardBot Team
Team Members: 	
	Liulu Yue	liuluyue@brandeis.edu
	Karen Mai	karenmai@brandeis.edu
	Rongzi Xie	rongzixie@brandeis.edu

2.We want to build a “GuardBot Team”, which includes multiple GuardBots and can be used to guard objects. Each GuardBot is able to recognize the target object, find the object and patrol around it, detect intruders, block the intruders to guard the object, and go to the owner of the object to notify intruding if the intruder would not go away after certain time period. 

On the showcase day, we want to have four GuardBots to form our GuardBot Team, an object that is able to move (eg. a toy car), some objects as obstacles in the room, a person playing the role of owner, and another person playing the role of the intruder. We want to demonstrate that GuardBots are able to recognize the object that needs to be guarded from all other objects in the room, and can go to the object and patrol around it without crashing into other obstacles. Then we will let the object move to see if GuardBots are still able to patrol around it. After the GuardBots patrol around the object for a while, we will have the intruder go to the object and try to grab it. GuardBots are expected to form a line in front of the intruder and block his way to the robot. The intruder would still want to go to the object, so we will see one of the GuardBot will find the way to the owner and notify him/her that the object is “in danger”.     
      
The basis of this project is to let 4 robots run on a single Ros core and we can have access to all of them.
TF2: TF2 is an important way to connect them and let them receive each other’s position and callback from LaserScan or Camera by transformation.
Map: Map considered to be a way to let robots be familiar with the environment to avoid hitting obstacles. We want our robots to move safely not only in the gazebo but in the real world with a random environment.
PID: PID will help our robot to move straight when they need to. For example, when they are patrolling, we want them to move straight without being away from the desired path to avoid hitting the object they protect.
We want our robot to block the intruder in front of it and when they are moving, they should move around from the object they protect and their colleagues. When the object moves away from the position it should be, the robot should follow it and try to block it by forming a circle again and start patrolling around it. We expect we can master these topics after finishing this project

There are some points we want to achieve:

1.     Block at desired position and strategy
2.     Don’t hit each other or obstacles
3.     Have some extra strategies
4.     The job is done with high efficiency(high speed and high accuracy)
5.     Never lose the target object
