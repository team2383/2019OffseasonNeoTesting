package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.modifiers.TankModifier;

import static frc.robot.HAL.drive;
import static frc.robot.HAL.navX;

import frc.robot.ninjaLib.PathFollower;
import frc.robot.ninjaLib.ReflectingCSVWriter;
import frc.robot.RobotMap;
import frc.robot.Robot;

public class FollowTrajectory extends Command implements Sendable  {
	PathFollower leftFollower;
	PathFollower rightFollower;
	Supplier<Trajectory> trajectorySupplier;
	Trajectory trajectory;
	TankModifier modifier;
	double angleDifference;
	boolean backwards;
	double startingAngle;
	
	double leftOutput;
	double rightOutput;
	
	double followerLoopTime;
	double followerdt;
	
	
	double lastTime;
	private Notifier notifier;
	
	public class DebugInfo {
		//public double leftPosition;
		//public double leftVelocity;
		public double leftPositionError;
		public double leftVelocityError;
		public double leftOutput;
		public double leftTargetVelocity;
		public double leftTargetAcceleration;
		
		//public double rightPosition;
		//public double rightVelocity;
		public double rightPositionError;
		public double rightVelocityError;
		public double rightOutput;
		public double rightTargetVelocity;
		public double rightTargetAcceleration;
		
		public double dt;
		public double loopTime;
	}
	
	private ReflectingCSVWriter<DebugInfo> writer;
	private DebugInfo debugInfo;
	
	public FollowTrajectory(Supplier<Trajectory> trajectorySupplier) {
		this(trajectorySupplier, false, 0);
	}
	
	public FollowTrajectory(Trajectory trajectory) {
		this(() -> trajectory, false, 0);
	}
	
	public FollowTrajectory(Trajectory trajectory, boolean backwards) {
		this(() -> trajectory, backwards, 0);
	}
	
	public FollowTrajectory(Supplier<Trajectory> trajectorySupplier, double startingAngle) {
		this(trajectorySupplier, false, startingAngle);
	}
	
	public FollowTrajectory(Trajectory trajectory, double startingAngle) {
		this(() -> trajectory, false, startingAngle);
	}
	
	public FollowTrajectory(Trajectory trajectory, boolean backwards, double startingAngle) {
		this(() -> trajectory, backwards, startingAngle);
	}
	 
	/**
	 * 
	 * @param trajectorySupplier
	 * @param backwards
	 * @param startingAngle RADIANS!
	 */
	public FollowTrajectory(Supplier<Trajectory> trajectorySupplier, boolean backwards, double startingAngle) {
		super("Follow Trajectory");

		this.trajectorySupplier = trajectorySupplier;
		this.backwards = backwards;
		this.startingAngle = startingAngle;
		
		requires(drive);
		
		
	}

	@Override
	protected void initialize() {
		notifier = new Notifier(this::notifierExecute);
		this.trajectory = trajectorySupplier.get();
		this.modifier = new TankModifier(trajectory).modify(RobotMap.kDrive_Motion_trackwidth);
		modifier.modify(RobotMap.kDrive_Motion_trackwidth);
		
		leftFollower = new PathFollower(modifier.getLeftTrajectory());
		rightFollower = new PathFollower(modifier.getRightTrajectory());
		
		leftFollower.configurePIDVA(RobotMap.kDrive_Motion_P,
				0.0,
				RobotMap.kDrive_Motion_D,
				RobotMap.kDrive_Motion_V,
				RobotMap.kDrive_Motion_A);

		rightFollower.configurePIDVA(RobotMap.kDrive_Motion_P,
				0.0,
				RobotMap.kDrive_Motion_D,
				RobotMap.kDrive_Motion_V,
				RobotMap.kDrive_Motion_A);

		leftFollower.reset();
		rightFollower.reset();
		drive.resetEncoders();
    	navX.zeroYaw();
    	
    	leftOutput = 0;
    	rightOutput = 0;
    	lastTime = Timer.getFPGATimestamp();
    	
    	followerLoopTime = 0;
    	followerdt = 0;
    	
    	debugInfo = new DebugInfo();
    	writer = new ReflectingCSVWriter<DebugInfo>("/home/lvuser/pathlog.csv", DebugInfo.class);
    	
    	notifier.startPeriodic(0.01);
	}
	
	@Override
	protected void execute() {		
		SmartDashboard.putNumber("MP Left Position Error (ft)", leftFollower.getError());
		SmartDashboard.putNumber("MP Right Position Error (ft)", rightFollower.getError());
		
		if(!backwards) {
			SmartDashboard.putNumber("MP Left Output (%)", leftOutput);
			SmartDashboard.putNumber("MP Right Output (%)", rightOutput);
		} else {
			SmartDashboard.putNumber("MP Left Output (%)", -rightOutput);
			SmartDashboard.putNumber("MP Right Output (%)", -leftOutput);
		}
		
		SmartDashboard.putNumber("MP Target Vel (ft-s)", leftFollower.getSegment().velocity);
		SmartDashboard.putNumber("MP Target Accel (ft-ss)", leftFollower.getSegment().acceleration);
		
		SmartDashboard.putNumber("follower dt", followerdt);
		SmartDashboard.putNumber("follower looptime", followerLoopTime);
		
		writer.flush();
	}

	private void notifierExecute() {
		double time = Timer.getFPGATimestamp();
		followerdt = (time - lastTime);
		lastTime = time;
		
		debugInfo.leftPositionError = leftFollower.getError();
		debugInfo.leftVelocityError = drive.getLeftVelocity() - leftFollower.getSegment().velocity;
		debugInfo.leftOutput = leftOutput;
		debugInfo.leftTargetVelocity = leftFollower.getSegment().velocity;
		debugInfo.leftTargetAcceleration = leftFollower.getSegment().acceleration;
		
		debugInfo.rightPositionError = rightFollower.getError();
		debugInfo.rightVelocityError = drive.getRightVelocity() - rightFollower.getSegment().velocity;
		debugInfo.rightOutput = rightOutput;
		debugInfo.rightTargetVelocity = rightFollower.getSegment().velocity;
		debugInfo.rightTargetAcceleration = rightFollower.getSegment().acceleration;
		
		debugInfo.dt = followerdt;
		debugInfo.loopTime = followerLoopTime;
		
		writer.add(debugInfo);
		
		double gyro_heading = -navX.getAngle(); //axis is the same
		
		double desired_heading = Pathfinder.r2d(leftFollower.getHeading() - startingAngle);  // Should also be in degrees, make sure its in phase
		
		angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
		
		double turn = RobotMap.kDrive_Motion_turnP * angleDifference;

		/*
		 * untested safety code. test on practice field b
		if (Math.abs(angleDifference) >= Constants.kAutoSafety_MaxGyroError ||
			Math.abs(leftFollower.getError()) >= Constants.kAutoSafety_MaxEncoderError ||
			Math.abs(rightFollower.getError()) >= Constants.kAutoSafety_MaxEncoderError) {
			System.out.println("!!!!!!!! AUTO FAILED !!!!!!!!");
		}
		
		*/
		
		
		if (!this.isFinished()) {
			
			//forwards
			if (!backwards) {
				leftOutput = leftFollower.calculate(drive.getLeftPosition());
				rightOutput = rightFollower.calculate(drive.getRightPosition());
			} else {
				//backwards
				leftOutput = leftFollower.calculate(-drive.getRightPosition()); //left = -right
				rightOutput = rightFollower.calculate(-drive.getLeftPosition()); //right = -left
			}
			
			if (!backwards) {
				drive.tank(leftOutput - turn, rightOutput + turn);
			} else {
				//backwards
				/*.tank is forwards, so (fwd_left, fwd_right)
				 * back_left = -fwd_right
				 * 	so fwd_right = -back_left
				 * back_right = -fwd_left
				 * 	so fwd_left = -back_right
				 * 
				 * turn input is relative to true (fwd) drivetrain output, not the actual direction
				 * so fwd_left(back_right) has to be less negative (going slower) then fwd_right(back_left), when turning right (negative turn)
				 */
				drive.tank(-rightOutput - turn, -leftOutput + turn);
			}
		}
		followerLoopTime = (Timer.getFPGATimestamp() - time);
	}

	@Override
	protected boolean isFinished() {
		return leftFollower.isFinished() && rightFollower.isFinished();
	}

	@Override
	protected void end() {
		System.out.println("pathDone");
		notifier.stop();
		
		drive.tank(0, 0);
    	
    	leftOutput = 0;
    	rightOutput = 0;
		
    	
		leftFollower.reset();
		rightFollower.reset();
    	navX.zeroYaw();
    	drive.resetEncoders();

    	notifier.stop();
	}

	@Override
	protected void interrupted() {
		end();
	}
}