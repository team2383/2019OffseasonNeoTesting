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

import frc.robot.ninjaLib.PathFollowerTalon;
import frc.robot.ninjaLib.ReflectingCSVWriter;
import frc.robot.RobotMap;

public class FollowTrajectoryTalon extends Command implements Sendable  {
	PathFollowerTalon leftFollower;
	PathFollowerTalon rightFollower;
	Supplier<Trajectory> trajectorySupplier;
	Trajectory trajectory;
	TankModifier modifier;
	double angleDifference;
	boolean backwards;
	double startingAngle;
	
	double leftSetpoint;
	double rightSetpoint;
	double leftFeedforward;
	double rightFeedforward;
	
	double followerLoopTime;
	double followerdt;
	
	
	double lastTime;
	private Notifier notifier;
	
	public class DebugInfo {
		//public double leftPosition;
		//public double leftVelocity;
		public double leftPositionError;
		public double leftVelocityError;
		public double leftSetpoint;
		public double leftTargetVelocity;
		public double leftTargetAcceleration;
		
		//public double rightPosition;
		//public double rightVelocity;
		public double rightPositionError;
		public double rightVelocityError;
		public double rightSetpoint;
		public double rightTargetVelocity;
		public double rightTargetAcceleration;
		
		public double dt;
		public double loopTime;
	}
	
	private ReflectingCSVWriter<DebugInfo> writer;
	private DebugInfo debugInfo;
	
	public FollowTrajectoryTalon(Supplier<Trajectory> trajectorySupplier) {
		this(trajectorySupplier, false, 0);
	}
	
	public FollowTrajectoryTalon(Trajectory trajectory) {
		this(() -> trajectory, false, 0);
	}
	
	public FollowTrajectoryTalon(Trajectory trajectory, boolean backwards) {
		this(() -> trajectory, backwards, 0);
	}
	
	public FollowTrajectoryTalon(Supplier<Trajectory> trajectorySupplier, double startingAngle) {
		this(trajectorySupplier, false, startingAngle);
	}
	
	public FollowTrajectoryTalon(Trajectory trajectory, double startingAngle) {
		this(() -> trajectory, false, startingAngle);
	}
	
	public FollowTrajectoryTalon(Trajectory trajectory, boolean backwards, double startingAngle) {
		this(() -> trajectory, backwards, startingAngle);
	}
	 
	/**
	 * 
	 * @param trajectorySupplier
	 * @param backwards
	 * @param startingAngle RADIANS!
	 */
	public FollowTrajectoryTalon(Supplier<Trajectory> trajectorySupplier, boolean backwards, double startingAngle) {
		super("Follow Trajectory");

		this.trajectorySupplier = trajectorySupplier;
		this.backwards = backwards;
		this.startingAngle = startingAngle;
		
		requires(drive);
		
		notifier = new Notifier(this::notifierExecute);
	}

	@Override
	protected void initialize() {
		this.trajectory = trajectorySupplier.get();
		this.modifier = new TankModifier(trajectory).modify(RobotMap.kDrive_Motion_trackwidth);
		modifier.modify(RobotMap.kDrive_Motion_trackwidth);
		
		leftFollower = new PathFollowerTalon(modifier.getLeftTrajectory());
		rightFollower = new PathFollowerTalon(modifier.getRightTrajectory());

		leftFollower.reset();
		rightFollower.reset();
		drive.resetEncoders();
    	navX.zeroYaw();
    	
    	leftSetpoint = 0;
    	rightSetpoint = 0;
    	lastTime = Timer.getFPGATimestamp();
    	
    	followerLoopTime = 0;
    	followerdt = 0;
    	
    	debugInfo = new DebugInfo();
    	writer = new ReflectingCSVWriter<DebugInfo>("/home/lvuser/pathlog.csv", DebugInfo.class);
    	
    	notifier.startPeriodic(0.01);
	}
	
	@Override
	protected void execute() {
		SmartDashboard.putNumber("MPT Left Setpoint", leftSetpoint);
		SmartDashboard.putNumber("MPT Right Setpoint", rightSetpoint);
		
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
		debugInfo.leftSetpoint = leftSetpoint;
		debugInfo.leftTargetVelocity = leftFollower.getSegment().velocity;
		debugInfo.leftTargetAcceleration = leftFollower.getSegment().acceleration;
		
		debugInfo.rightPositionError = rightFollower.getError();
		debugInfo.rightVelocityError = drive.getRightVelocity() - rightFollower.getSegment().velocity;
		debugInfo.rightSetpoint = rightSetpoint;
		debugInfo.rightTargetVelocity = rightFollower.getSegment().velocity;
		debugInfo.rightTargetAcceleration = rightFollower.getSegment().acceleration;
		
		debugInfo.dt = followerdt;
		debugInfo.loopTime = followerLoopTime;
		
		writer.add(debugInfo);
		
		double gyro_heading = -navX.getAngle(); //axis is the same
		
		double desired_heading = Pathfinder.r2d(leftFollower.getHeading() - startingAngle);  // Should also be in degrees, make sure its in phase
		
		angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
		
		double turn = RobotMap.kDrive_Motion_turnP * angleDifference;
		
		leftFeedforward = leftFollower.getSegment().velocity * RobotMap.kDrive_Motion_V;
		rightFeedforward = rightFollower.getSegment().velocity * RobotMap.kDrive_Motion_V;

		leftSetpoint = leftFollower.calculate();
		rightSetpoint = rightFollower.calculate();
		
		followerLoopTime = (Timer.getFPGATimestamp() - time);
		
		if(!this.isFinished()) {
			if(!backwards) {
				drive.positionPDauxF(leftSetpoint, leftFeedforward - turn, rightSetpoint, rightFeedforward + turn);
			} else {
				drive.positionPDauxF(-rightSetpoint, leftFeedforward - turn, -leftSetpoint, rightFeedforward + turn);
			}
		}
	}

	@Override
	protected boolean isFinished() {
		return leftFollower.isFinished() && rightFollower.isFinished() && drive.atTarget();
	}

	@Override
	protected void end() {
		System.out.println("pathDone");
		drive.tank(0, 0);
    	
    	leftSetpoint = 0;
    	rightSetpoint = 0;
		
		leftFollower.reset();
		rightFollower.reset();
    	navX.zeroYaw();
    	
    	notifier.stop();
	}

	@Override
	protected void interrupted() {
		end();
	}
}