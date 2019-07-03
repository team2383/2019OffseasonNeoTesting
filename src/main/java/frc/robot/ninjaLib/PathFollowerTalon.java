package frc.robot.ninjaLib;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

/**
 * The DistanceFollower is an object designed to follow a trajectory based on distance covered input. This class can be used
 * for Tank or Swerve drive implementations.
 *
 * @author Jaci
 */
public class PathFollowerTalon {
	double last_error, heading;

    int segment;
    Trajectory trajectory;    
    TalonData td;
    
    public class TalonData {
    	public double positionSetpoint;
    	public double auxFeedForward;
    }

    public PathFollowerTalon(Trajectory traj) {
        this.trajectory = traj;
        td = new TalonData();
    }

    public PathFollowerTalon() {
    	td = new TalonData();
    }

    /**
     * Set a new trajectory to follow, and reset the cumulative errors and segment counts
     */
    public void setTrajectory(Trajectory traj) {
        this.trajectory = traj;
        reset();
    }

    /**
     * Reset the follower to start again. Encoders must be reconfigured.
     */
    public void reset() {
        last_error = 0; segment = 0;
    }

    /**
     * Calculate the desired output for the motors, based on the distance the robot has covered.
     * This does not account for heading of the robot. To account for heading, add some extra terms in your control
     * loop for realignment based on gyroscope input and the desired heading given by this object.
     * @param distance_covered  The distance covered in meters
     * @return                  The desired output for your motor controller
     */
    public double calculate() {
        if (segment < trajectory.length()) {
            return trajectory.get(segment++).position;
        } else {
        	return 0;
        }
    }

    /**
     * @return the desired heading of the current point in the trajectory
     */
    public double getHeading() {
        return heading;
    }

    /**
     * @return the last error of the PID loop
     */
    public double getError() {
    	return last_error;
    }
    
    /**
     * @return the current segment being operated on
     */
    public Trajectory.Segment getSegment() {
    	Trajectory.Segment seg = new Trajectory.Segment(0, 0, 0, 0, 0, 0, 0, 0);
    	try {
    		seg = trajectory.get(segment);
    	} catch (ArrayIndexOutOfBoundsException e){
    		//lol bye
    	}
        return seg;
    }

    /**
     * @return whether we have finished tracking this trajectory or not.
     */
    public boolean isFinished() {
        return segment >= trajectory.length();
    }

}

