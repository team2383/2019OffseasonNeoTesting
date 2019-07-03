/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import frc.robot.ninjaLib.PathLoader;
import frc.robot.commands.FollowTrajectory;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

// (X,Y,THETA)
public class BaselineAuto extends CommandGroup {
	Waypoint[] baseline = new Waypoint[] {
			new Waypoint(0, 0, 0),
			new Waypoint(4, 0, 0)
			};

	Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
			0.02, 	//delta time
			2,		//max velocity in ft/s for the motion profile
			0.8,		//max acceleration in ft/s/s for the motion profile
			2.5);	//max jerk in ft/s/s/s for the motion profile

	Trajectory trajectory = PathLoader.get(baseline, config);

	public BaselineAuto(boolean backwards) {
		backwards = true;
		addSequential(new FollowTrajectory(trajectory, backwards));
	}
}

