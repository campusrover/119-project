# GuardBot Team
## Team Members
Liulu Yue	liuluyue@brandeis.edu<br />
Karen Mai	karenmai@brandeis.edu<br />
Rongzi Xie	rongzixie@brandeis.edu<br />
## Long Term Goal
* We want to build a “*GuardBot* Team”, which includes multiple *GuardBots* and can be used to guard objects. Each GuardBot is able to recognize the target object, find the object and patrol around it, detect intruders, block the intruders to guard the object, and go to the owner of the object to notify intruding if the intruder would not go away after certain time period. 

On the showcase day, we want to have four *GuardBots* to form our *GuardBot* Team, an object that is able to move (eg. a toy car), some objects as obstacles in the room, a person playing the role of owner, and another person playing the role of the intruder. We want to demonstrate that *GuardBots* are able to recognize the object that needs to be *guarded* from all other objects in the room, and can go to the object and patrol around it without crashing into other obstacles. Then we will let the object move to see if *GuardBots* are still able to patrol around it. After the *GuardBots* patrol around the object for a while, we will have the intruder go to the object and try to grab it. *GuardBots* are expected to form a line in front of the intruder and block his way to the robot. The intruder would still want to go to the object, so we will see one of the *GuardBot* will find the way to the owner and notify him/her that the object is “in danger”.     
      
The basis of this project is to let 4 robots run on a single Ros core and we can have access to all of them.
* TF2: TF2 is an important way to connect them and let them receive each other’s position and callback from LaserScan or Camera by transformation.
* Map: Map considered to be a way to let robots be familiar with the environment to avoid hitting obstacles. We want our robots to move safely not only in the gazebo but in the real world with a random environment.
* PID: PID will help our robot to move straight when they need to. For example, when they are patrolling, we want them to move straight without being away from the desired path to avoid hitting the object they protect.
We want our robot to block the intruder in front of it and when they are moving, they should move around from the object they protect and their colleagues. When the object moves away from the position it should be, the robot should follow it and try to block it by forming a circle again and start patrolling around it. We expect we can master these topics after finishing this project
## Short Term Goal
1.     Block at desired position and strategy
2.     Don’t hit each other or obstacles
3.     Have some extra strategies
4.     The job is done with high efficiency(high speed and high accuracy)
5.     Never lose the target object
## Current Structure
* Target: The object or moving creatures that need protection
* Intruder: A moving enemy to the target
* Guards: Several robots that will protect the target by patroling and blocking the intruders
### Launch Files:
We have launch files for multiple robots, single robot, small object and a world with a small cube in the middle as the target object. We currently have four robots on gazebo.
### Nodes:
* Guard_Broadcaster: A broadcaster that send odom msg transformation that can get odom msg(position, rotation) from *guards*.
* RevelveAroundObject: An algorithm that currently can let four robots patrolling around an object. It also contains other algorithm including going to a desired position, find object, drive to intruder and block the inturder and etc that remain untested.
*
### Worlds
Set up of the world in use of testing in bazebo. Contain a simple world with a small box as the target.
## New thoughts in current stage:
* Based on what Karen did last week, we think we can have not only four robots, more robots, bigger space of partolling area and more accurate position message can make the guards be more powerful and accurate.
Based on this thought, we can develop new strategy of handling multiple intruders.
1.    While several robots are blocking one intruder, a guard will keep patrolling to avoid multiple intruders.
2.    After the first inturder begin to move closer or perform more threatening actions, it will go back with its colleagues to focus on the one intruder
* New type of robot:<br />
  **Operator**: After finding the inturder, it will go to a specific position point and try to call for help(bring human back to the target)
