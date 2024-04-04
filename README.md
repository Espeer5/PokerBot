## POKERBOT
A passion Project by Edward Speer, Leo Jenkins, Joy Liu, and Josh Roberts
Special thanks to Gunter Niermeyer and the Caltech WI24 ME/CS/EE 134 TA Team

## Description:
PokerBot is a 5 DOF robotic arm with speaker and vacuum systems, which plays 
Texas Hold 'em poker against 1-3 human players within a semicircular workspace
of around .6 meters. PokerBot is fully autonomous, requiring no human
intervention to play a full poker match.

The PokerBot system consists of a robotic arm armed with a suction end effector,
which may be used to grab, rotate, drop, and flip playing cards and poker chips.
There is a ceiling mounted camera feed which uses OpenCV computer vision routines 
to analyze the game state and inform the game logic. There is also an aux speaker 
connected which is used to prompt players to play the game correctly using python 
text-to-speech libraries. Finally, there is an additional camera mounted within 
the robot card box so that the robot may identify what its own cards are.

PokerBot plays according to a simple strategy given by its betting model. The 
betting model considers the identity of th community cards and its own cards, 
and scores its own hand vs. an average hand according to the likelihood of 
winning. Then, dependent on a free risk parameter, the robot determines whether 
to call, raise, or fold dependent on the delta between its own hand score and 
that of an average hand.

## Software Structure:
The PokerBot robotic system operates as a series of networked ROS2 nodes which may be 
run across multiple devices. Each required CV detector (for chips, backs of cards, 
front of cards, and the betting button) has it's own (computationall expensive)
node. There is then a brain node, responsible for deciding which action to take 
based on the observed game state as well as solving for joint angles in simulation.
There is then a hardware control node, responsible for publishing motor positions
and vacuum system state commands at a very high rate to the hardware to ensure 
fluid and accurate movements.

## Kinematic Controller:
This robot is designed to interact closely with human opponents in a way which 
is quick, and within a small workspace. To mitigate safety concerns having to 
do with these constraints, the robot needs a very fast and lightweight
hardware controller node which achieves high precision and fluidity. In order 
to ensure this is possible, the inverse kinematics, requiring matrix inversion 
and other canonically slow operations, are solved in simulation. This also avoids
concerns with singularities and solution multiplicities - For each simulated run 
solving for joint positions, the arm is seeded in the center of the elbow-up 
solution, meaning the solved joints with sufficiently small simulated timesteps
will always be in the elbow-up range (and therefore avoid interference with the
playing table surface). In order to avoid issues with jitter near singularities, 
a weighted psuedoinverse is used which dampens any velocity to 0 near the 
boundaries of the arm's workspace.

For any more information about this project, feel free to reach out. This 
project was a vast amount of work that we take pride in, so we are always happy
to discuss!
