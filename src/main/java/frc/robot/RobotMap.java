package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  // DRIVETRAIN
  public static int rightMasterPort = 1;
  public static int rightFollowerPort = 2;
  public static int leftMasterPort = 3;
  public static int leftFollowerPort = 5;

  // CLOCK SUBSYSTEM
  public static int clockPort = 8;
  

  // CONSTANTS
  public static double kDrive_Motion_P = 1.4;				// %/ft
  public static double kDrive_Motion_D = 0.0;	
  public static double kDrive_Motion_A = 0.0;	
  public static double kDrive_Motion_V = 0.058;

  public static double kDrive_Motion_trackwidth = 2.72;
  public static double kDrive_Motion_turnP = 0.0175;
  
  public static double kDrive_Motion_Tolerance = 0.05;// ft
  public static double kDrive_WheelDiameterInch = 3.875;
  public static double getWheelCircumference() { return (kDrive_WheelDiameterInch*Math.PI)/12.0; };

  public static int kDrive_ContinuousCurrentLimit = 60;
	public static int kDrive_PeakCurrentLimit = 80;
  public static int kDrive_PeakCurrentTime_ms = 100;
  
  public static double kDrive_peakOutput = 0.8;
  public static double kDrive_Motion_talonP = 0.7;			// %/ft
	public static double kDrive_Motion_talonI = 0.002;			//natives
  public static double kDrive_Motion_talonD = 15;
  
  	//talon V and motio V are shared
	public static double kDrive_Motion_Velocity = 6.0;		// for turn
	public static double kDrive_Motion_Acceleration = 13.0;
}
