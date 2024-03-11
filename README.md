Known issues:
 - Cannot pick up button in top right corner
 - Incorrectly quantifies value of pot/bet
 - Face up location is very tight
 - Minimum threshold for FOC detection is too low (sees BC as FOC)
 
Known tasks:
 - Second camera
 - Filter button location
 - Make debug publisher for each detector
 - Robot presents cards in showdown
 - Arm sends message when action is over
 - Write payout code
    - Robot collects own chips?
 - After every task, check if the task was successful
 - Contact detection


Leo's To-Do List:
 - Improve chip detection
   - Expand HSV range for non-black chips?
   - Re-take reference images?
   - Check size and feature matching bounds
 - Improve FOC detection
   - Increase lower bound (stop detecting BOC)
   - Re-take reference images?
   - Add sampling?
 - Update chip box locations (once 3d prints are done)
 - Figure out a way to indicate that the robot has folded
 - Work with Eddie to integrate robot decision making with betting code
