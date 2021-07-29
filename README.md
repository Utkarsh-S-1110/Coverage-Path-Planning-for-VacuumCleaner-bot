# VacuumCleaner
-This is a project for probation period of 'Team Robocon'
-To test the path planning code for any shortcomings, please run the file in the Test_without_simulation folder. It will print the co-ordinates of all the grids that the bot will travel to cover the desired area.
-Please change the values in the main method of the function to set up different scenarios for the bot
-In the simulation in the test environment, the movement of the robots are not as precise as we want due to unaccounted slipping of the bot and inaccuracy in the odometric data.
 This could sometimes lead to the bot to slightly bump into some obstacles or not go as close to the obstacle as desired
-To reduce slipping, we have given very less linear velocity to the bot due to which the simulation may go on for a long time.
-We are still working on integrating the cad model in the desired environment. Being smaller in size to the turtlebot, it would lead to the robot covering less distance and thus, lead to less deviation or error.
