package frc.robot.ninjaLib;

import jaci.pathfinder.Trajectory;

/**
 * The DistanceFollower is an object designed to follow a trajectory based on distance covered input. This class can be used
 * for Tank or Swerve drive implementations.
 *
 * @author Jaci
 */
public class PathFollower {

    double kp, ki, kd, kv, ka;

	double last_error, heading;

    int segment;
    Trajectory trajectory;

    public PathFollower(Trajectory traj) {
        this.trajectory = traj;
    }

    public PathFollower() { }

    /**
     * Set a new trajectory to follow, and reset the cumulative errors and segment counts
     */
    public void setTrajectory(Trajectory traj) {
        this.trajectory = traj;
        reset();
    }

    /**
     * Configure the PID/VA Variables for the Follower
     * @param kp The proportional term. This is usually quite high (0.8 - 1.0 are common values)
     * @param ki The integral term. Currently unused.
     * @param kd The derivative term. Adjust this if you are unhappy with the tracking of the follower. 0.0 is the default
     * @param kv The velocity ratio. This should be 1 over your maximum velocity @ 100% throttle.
     *           This converts m/s given by the algorithm to a scale of -1..1 to be used by your
     *           motor controllers
     * @param ka The acceleration term. Adjust this if you want to reach higher or lower speeds faster. 0.0 is the default
     */
    public void configurePIDVA(double kp, double ki, double kd, double kv, double ka) {
        this.kp = kp; this.ki = ki; this.kd = kd;
        this.kv = kv; this.ka = ka;
    }

    public double getKp() {
		return kp;
	}

	public void setKp(double kp) {
		this.kp = kp;
	}

	public double getKi() {
		return ki;
	}

	public void setKi(double ki) {
		this.ki = ki;
	}

	public double getKd() {
		return kd;
	}

	public void setKd(double kd) {
		this.kd = kd;
	}

	public double getKv() {
		return kv;
	}

	public void setKv(double kv) {
		this.kv = kv;
	}

	public double getKa() {
		return ka;
	}

	public void setKa(double ka) {
		this.ka = ka;
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
    public double calculate(double distance_covered) {
        if (segment < trajectory.length()) {
            Trajectory.Segment seg = trajectory.get(segment);
            double error = seg.position - distance_covered;
            
            /*
             * pwm = P * error + D * ((error - prev_error) / dt - goal_velocity) + Kv * goal_velocity + Ka * goal_acceleration
			 * This is close to a PD controller, but with some feed forward terms. The idea behind feed forwards is that if you have a pretty good idea
			 * about how much power it will take to do something, go ahead and add it in. The control loop will take up the slop from there when you are wrong,
			 * or someone bumped the bot. The D term is special in that you subtract off the goal velocity (should we do this). This falls out from the state feedback controllers.
			 * Think about it this way. If you are at the goal, and moving at the right speed, you don't want to apply corrective power to decelerate the robot
			 * (This is what D would do if you weren't trying to move but were moving and at the goal.)
             */
            
            double calculated_value =
                    kp * error +                                    // Proportional
                    kd * ((error - last_error) / seg.dt - seg.velocity) +          // Derivative
                    (kv * seg.velocity + ka * seg.acceleration);    // V and A Terms
            last_error = error;
            heading = seg.heading;
            segment++;

            return calculated_value;
        } else return 0;
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
