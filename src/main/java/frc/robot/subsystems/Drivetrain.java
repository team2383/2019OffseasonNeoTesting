package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;



import frc.robot.ninjaLib.MotionUtils;
import frc.robot.ninjaLib.Values;
import frc.robot.RobotMap;

import static frc.robot.HAL.navX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.DriveCommand;

public class Drivetrain extends Subsystem {

  public final WPI_TalonSRX leftMaster;
	public final BaseMotorController leftFollowerA;
	public final BaseMotorController leftFollowerB;
	public final BaseMotorController leftFollowerC;

	public final WPI_TalonSRX rightMaster;
	public final BaseMotorController rightFollowerA;
	public final BaseMotorController rightFollowerB;
  public final BaseMotorController rightFollowerC;
  
  public final DifferentialDrive drive;

  public Drivetrain (){
    super("Drivetrain");

    ////////////////
    // LEFT DRIVE //
    ////////////////
    
    leftMaster = new WPI_TalonSRX(47);
    // ALPHA BOT
    leftFollowerA = new TalonSRX(80);
	leftFollowerB = new TalonSRX(69);
    leftFollowerC = new TalonSRX(42);

    // OMEGA BOT
    // leftFollowerA = new VictorSPX(2);
	//   leftFollowerB = new VictorSPX(3);
    // leftFollowerC = new VictorSPX(4);

    leftFollowerA.follow(leftMaster);
		leftFollowerB.follow(leftMaster);
		leftFollowerC.follow(leftMaster);
    
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		leftMaster.setNeutralMode(NeutralMode.Brake);
		leftMaster.enableVoltageCompensation(true);
		leftMaster.configVoltageCompSaturation(12, 10);

		//clear options
		leftMaster.configForwardSoftLimitEnable(false, 10);
		leftMaster.configReverseSoftLimitEnable(false, 10);
		
		leftMaster.setControlFramePeriod(ControlFrame.Control_3_General, 5);
		leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10);

		leftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_20Ms, 10);

    /////////////////
    // RIGHT DRIVE //
    /////////////////
    rightMaster = new WPI_TalonSRX(5);
    // ALPHA BOT
    rightFollowerA = new TalonSRX(6);
	rightFollowerB = new TalonSRX(7);
    rightFollowerC = new TalonSRX(8);
    
    // OMEGA BOT
    // rightFollowerA = new VictorSPX(6);
	// rightFollowerB = new VictorSPX(7);
    // rightFollowerC = new VictorSPX(8);

    rightFollowerA.follow(rightMaster);
	rightFollowerB.follow(rightMaster);
    rightFollowerC.follow(rightMaster);
    
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
	rightMaster.setNeutralMode(NeutralMode.Brake);
	rightMaster.enableVoltageCompensation(true);
	rightMaster.configVoltageCompSaturation(12, 10);
		
		//clear options
	rightMaster.configForwardSoftLimitEnable(false, 10);
	rightMaster.configReverseSoftLimitEnable(false, 10);
		
	rightMaster.setControlFramePeriod(ControlFrame.Control_3_General, 5);
	rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10);
		
	rightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_20Ms, 10);

    configMotorControllers(10);
    
		
		setBrake(true);
    drive = new DifferentialDrive(leftMaster, rightMaster);
    drive.setSafetyEnabled(false);
		drive.setMaxOutput(1.0);
  }

  public void configMotorControllers(int timeout) {
		double kWheelCircumference = RobotMap.getWheelCircumference();

		// ALPHA BOT
		leftMaster.setSensorPhase(false);
		leftMaster.setInverted(true); // change here
		leftFollowerA.setInverted(false);
		leftFollowerB.setInverted(false);
		leftFollowerC.setInverted(false);

		// OMEGA BOT
		// leftMaster.setSensorPhase(false);
		// leftMaster.setInverted(true);
		// leftFollowerA.setInverted(true);
		// leftFollowerB.setInverted(false);
		// leftFollowerC.setInverted(true);

		leftMaster.configSetParameter(ParamEnum.eContinuousCurrentLimitAmps, RobotMap.kDrive_ContinuousCurrentLimit, 0x00, 0x00, timeout);
		leftMaster.configSetParameter(ParamEnum.ePeakCurrentLimitAmps, RobotMap.kDrive_PeakCurrentLimit, 0x00, 0x00, timeout);
		leftMaster.configSetParameter(ParamEnum.ePeakCurrentLimitMs, RobotMap.kDrive_PeakCurrentTime_ms, 0x00, 0x00, timeout);
		leftMaster.enableCurrentLimit(false);
		
		leftMaster.configPeakOutputForward(RobotMap.kDrive_peakOutput, timeout);
		leftMaster.configPeakOutputReverse(-RobotMap.kDrive_peakOutput, timeout);
		leftMaster.configOpenloopRamp(0.0, timeout);
		
		
		// ALPHA BOT
		rightMaster.setSensorPhase(false);
		rightMaster.setInverted(false);
		rightFollowerA.setInverted(false);
		rightFollowerB.setInverted(true);
		rightFollowerC.setInverted(true);

		// OMEGA BOT
		// rightMaster.setSensorPhase(false);
		// rightMaster.setInverted(true);
		// rightFollowerA.setInverted(true);
		// rightFollowerB.setInverted(true);
		// rightFollowerC.setInverted(true);
		
		rightMaster.configSetParameter(ParamEnum.eContinuousCurrentLimitAmps, RobotMap.kDrive_ContinuousCurrentLimit, 0x00, 0x00, timeout);
		rightMaster.configSetParameter(ParamEnum.ePeakCurrentLimitAmps, RobotMap.kDrive_PeakCurrentLimit, 0x00, 0x00, timeout);
		rightMaster.configSetParameter(ParamEnum.ePeakCurrentLimitMs, RobotMap.kDrive_PeakCurrentTime_ms, 0x00, 0x00, timeout);
		rightMaster.enableCurrentLimit(false);
		
		rightMaster.configPeakOutputForward(RobotMap.kDrive_peakOutput, timeout);
		rightMaster.configPeakOutputReverse(-RobotMap.kDrive_peakOutput, timeout);
		rightMaster.configOpenloopRamp(0.0, timeout);
		
		//PID
		rightMaster.config_kP(0, -(RobotMap.kDrive_Motion_talonP * (1023.0/1.0) * (1.0/(kWheelCircumference)) * (1.0/4096.0)), 10);
		rightMaster.config_kI(0, -RobotMap.kDrive_Motion_talonI, 10);
		rightMaster.config_kD(0, -(RobotMap.kDrive_Motion_talonD * (1023.0/1.0) * (1.0/(1.0/kWheelCircumference)) * (1.0/4096.0) * (10.0)), 10);
		rightMaster.config_kF(0, -(RobotMap.kDrive_Motion_V * (1023.0/1.0) * (1.0/(1.0/kWheelCircumference)) * (1.0/4096.0) * (10.0)),  10);
		rightMaster.config_IntegralZone(0, 50, 10);

		rightMaster.config_kP(1, -(RobotMap.kDrive_Motion_talonP * (1023.0/1.0) * (1.0/(kWheelCircumference)) * (1.0/4096.0)), 10);
		rightMaster.config_kI(1, -RobotMap.kDrive_Motion_talonI, 10);
		rightMaster.config_kD(1, -(RobotMap.kDrive_Motion_talonD * (1023.0/1.0) * (1.0/(1.0/kWheelCircumference)) * (1.0/4096.0) * (10.0)), 10);
		rightMaster.config_kF(1, 0,  10);
		rightMaster.config_IntegralZone(1, 50, 10);

		leftMaster.config_kP(0, (RobotMap.kDrive_Motion_talonP * (1023.0/1.0) * (1.0/(kWheelCircumference)) * (1.0/4096.0)), 10);
		leftMaster.config_kI(0, RobotMap.kDrive_Motion_talonI, 10);
		leftMaster.config_kD(0, (RobotMap.kDrive_Motion_talonD * (1023.0/1.0) * (1.0/(1.0/kWheelCircumference)) * (1.0/4096.0) * (10.0)), 10);
		leftMaster.config_kF(0, (RobotMap.kDrive_Motion_V * (1023.0/1.0) * (1.0/(1.0/kWheelCircumference)) * (1.0/4096.0) * (10.0)),  10);
		leftMaster.config_IntegralZone(0, 50, 10);
		
		leftMaster.config_kP(1, (RobotMap.kDrive_Motion_talonP * (1023.0/1.0) * (1.0/(kWheelCircumference)) * (1.0/4096.0)), 10);
		leftMaster.config_kI(1, RobotMap.kDrive_Motion_talonI, 10);
		leftMaster.config_kD(1, (RobotMap.kDrive_Motion_talonD * (1023.0/1.0) * (1.0/(1.0/kWheelCircumference)) * (1.0/4096.0) * (10.0)), 10);
		leftMaster.config_kF(1, 0,  10);
		leftMaster.config_IntegralZone(1, 50, 10);
		
		/*
		 * ft/s -> ticks per 100ms
		 * so ft/s * 1 rotation / circumference ft * 4096 / 1 rotation / 10
		 */
		int nativeVelocity = (int) (RobotMap.kDrive_Motion_Velocity * 1.0/RobotMap.getWheelCircumference() * 4096.0 / 10.0);

		/*
		 * ft/s/s -> ticks per 100ms per s
		 * so ft/s/s * 1 rotation / circumference ft * 4096 / 1 rotation / 10
		 */
		int nativeAcceleration = (int) (RobotMap.kDrive_Motion_Acceleration * 1.0/RobotMap.getWheelCircumference() * 4096.0 / 10.0);
		leftMaster.configMotionCruiseVelocity(nativeVelocity, timeout);
		leftMaster.configMotionAcceleration(nativeAcceleration, timeout);
		rightMaster.configMotionCruiseVelocity(nativeVelocity, timeout);
		rightMaster.configMotionAcceleration(nativeAcceleration, timeout);
	}

  public void arcade(double move, double turn){
    drive.arcadeDrive(move, turn);
  }

  public void tank(double left, double right)
  {
    drive.tankDrive(left, right);
  }

  public void resetEncoders() {
		leftMaster.setSelectedSensorPosition(0, 0, 0);
		rightMaster.setSelectedSensorPosition(0, 0, 0);
  }
  
  public double getLeftVelocity() {
		return leftMaster.getSelectedSensorVelocity(0) / 4096.0 * 10.0 * RobotMap.getWheelCircumference();
	}
	
	public double getRightVelocity() {
		return rightMaster.getSelectedSensorVelocity(0) / 4096.0 * 10.0 * RobotMap.getWheelCircumference();
  }
  
  public void positionPDauxF(double leftPos, double leftFF, double rightPos, double rightFF) {
		leftMaster.selectProfileSlot(1, 0);
		rightMaster.selectProfileSlot(1, 0);

		leftMaster.set(ControlMode.Position, MotionUtils.distanceToRotations(leftPos, RobotMap.getWheelCircumference()) * 4096, DemandType.ArbitraryFeedForward, leftFF);
		rightMaster.set(ControlMode.Position, MotionUtils.distanceToRotations(rightPos, RobotMap.getWheelCircumference()) * 4096, DemandType.ArbitraryFeedForward, -rightFF);
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveCommand(OI.throttle,OI.turn));
  }

  public double inches(int ticks) {
		return MotionUtils.rotationsToDistance(MotionUtils.ticksToRotations(ticks, 4096, 1), RobotMap.getWheelCircumference());
  }
  
  public boolean atTarget() {
		return Math.abs(inches(leftMaster.getClosedLoopError(0))) < RobotMap.kDrive_Motion_Tolerance && Math.abs(inches(leftMaster.getClosedLoopError(0))) < RobotMap.kDrive_Motion_Tolerance;
  }

  public double getLeftPosition() {
		return MotionUtils.rotationsToDistance(MotionUtils.ticksToRotations(leftMaster.getSelectedSensorPosition(0), 4096, 1), RobotMap.getWheelCircumference());
	}
	
	public double getRightPosition() {
		return MotionUtils.rotationsToDistance(MotionUtils.ticksToRotations(rightMaster.getSelectedSensorPosition(0), 4096, 1), RobotMap.getWheelCircumference());
  }
  
  public void setBrake(boolean brake) {
		NeutralMode mode = brake ? NeutralMode.Brake : NeutralMode.Coast;
		leftMaster.setNeutralMode(mode);
		leftFollowerA.setNeutralMode(mode);
		leftFollowerB.setNeutralMode(mode);
		leftFollowerC.setNeutralMode(mode);
		rightMaster.setNeutralMode(mode);
		rightFollowerA.setNeutralMode(mode);
		rightFollowerB.setNeutralMode(mode);
		rightFollowerC.setNeutralMode(mode);
	}
  
}