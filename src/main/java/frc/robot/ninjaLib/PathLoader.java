/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ninjaLib;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class PathLoader {
	public static Trajectory get(Waypoint[] points, Trajectory.Config config) {
		String pathHash = String.valueOf(generateHashCode(points, config));
		System.out.println("loading path " + pathHash + " with " + points.length + 
							" points, ends at (" + points[points.length-1].x + 
							", " + points[points.length-1].y + 
							", " + Math.toDegrees(points[points.length-1].angle) + 
							")");

		SmartDashboard.putString("Path Hash", pathHash);
		Trajectory trajectory;
		File trajectoryFile = new File("/home/lvuser/paths/" + pathHash + ".csv");
		System.out.println("path " + pathHash + " does not exist");
		trajectory = Pathfinder.generate(points, config);
		System.out.println("path " + pathHash + " generated");
		Pathfinder.writeToCSV(trajectoryFile, trajectory);
		System.out.println("path " + pathHash + " .csv not found, wrote to file");
		// if (!trajectoryFile.exists()) {
		// 	System.out.println("path " + pathHash + " does not exist");

		// 	trajectory = Pathfinder.generate(points, config);
			
		// 	System.out.println("path " + pathHash + " generated");
			
		// 	Pathfinder.writeToCSV(trajectoryFile, trajectory);

		// 	System.out.println("path " + pathHash + " .csv not found, wrote to file");
		// } else {
		// 	System.out.println("path " + pathHash + " found");

		// 	trajectory = Pathfinder.readFromCSV(trajectoryFile);

		// 	System.out.println("path " + pathHash + ".csv read from file");
		// }
		return trajectory;
	}

	private static String generateHashCode(Waypoint[] path, Trajectory.Config config) {
		double hash = 1.0;
		for (int i = 0; i < path.length; i++) {
			double r = 1;
			r = 7 * r + path[i].x;
			r = 31 * r + path[i].y;
			r = 13 * r + path[i].angle;
			hash += r;
		}

		hash += ((config.dt * 3) + (config.max_velocity * 7) + (config.max_acceleration * 11) + (config.max_jerk * 19)
				+ (config.fit.ordinal() * 7) + (config.sample_count * 23));
		
		return Long.toHexString(Double.doubleToRawLongBits(hash));
	}
}

