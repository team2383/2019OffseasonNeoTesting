package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.*;

public class Neo extends Subsystem {

  //public TalonSRX clock = new TalonSRX(10);
  public CANSparkMax neoMotor = new CANSparkMax(20, MotorType.kBrushless);
  public CANEncoder neoEncoder = new CANEncoder(neoMotor);
  public CANPIDController neoController = new CANPIDController(neoMotor);


  // Create preset positions with a name and a corresponding encoder value
  // These preset names are used in OI
  public static enum NeoPreset{
    MOVEIT(2);

    public double neoPosition;

    private NeoPreset(double neoPosition) {
        this.neoPosition = neoPosition;
      }
    }

  // Configures PID values, sets velocity & acceleration, determines in what direction is positive for the encoders
  public void configMotorController(int timeout){
      neoController.setP(1);
      neoController.setI(0);
      neoController.setD(0);
      neoController.setOutputRange(-1, 1);
      neoMotor.setCANTimeout(timeout);
  }  

  // Needed to call the configMotorController method above
  public Neo (){
    configMotorController(10);
  }

  // Set the default command for a subsystem here. 
  @Override
  public void initDefaultCommand() {  
    //setDefaultCommand(new SetClock(OI.clockManual));
  }

  // Able to return values and read them off of Shuffleboard or the SmartDashboard
  public void periodic(){
    SmartDashboard.putNumber("NEO Position", getCurrentPosition());
    SmartDashboard.putNumber("NEO Temp", getMotorTemp());
  }

  // Method that will return the neo's current position
  public double getCurrentPosition() {
    return neoEncoder.getPosition();
  }

  public double getMotorTemp(){
    return neoMotor.getMotorTemperature();
  }

  // Method that will set the position of the clock to a given position
  public void setPosition(NeoPreset position){
    neoController.setReference(100, ControlType.kPosition);
    //neoMotor.pidWrite();
    //neoMotor.pidWrite(neoController.);
    //neoMotor.pidWrite((neoController.getOutputMin() + neoController.getOutputMax())/2);
    //neoMotor.set(position.neoPosition);
  }

  // Method that will set the speed of the neo motor.
  // Speed ranges from 1 to -1 where 1 is 100% speed in one direction while -1 is 100% speed in the opposite direction
  public void setSpeed(double speed){
    neoMotor.set(speed);
  }

  // Method that will stop the neo motor when called
  public void stop() {
    neoMotor.set(0);
  }

}