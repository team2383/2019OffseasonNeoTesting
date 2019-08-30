package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import java.util.function.DoubleSupplier;

import frc.robot.commands.*;
import frc.robot.ninjaLib.Gamepad;
import frc.robot.subsystems.*;



/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a joystick.
  // You create one by telling it which joystick it's on and which button number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

   /////////////////////
  // XBOX CONTROLLER //
  /////////////////////

  public static Gamepad driver = new Gamepad(0);
  //public static Joystick clockJS = new Joystick(1);
  
  // DRIVE
  public static DoubleSupplier turn = () -> -(driver.getRightX());
  public static DoubleSupplier throttle = () -> (driver.getLeftY()); 
  public static DoubleSupplier clockManual = () -> (driver.getLeftTrigger());
  //public static DoubleSupplier clockManualBack = () -> (driver.getRightTrigger());

  // CLOCK CONTROLS
  //public static DoubleSupplier clock = () -> clockJS.getY(); 
  // public static Button twelve = new JoystickButton(driver, Gamepad.BUTTON_Y);
  // public static Button three = new JoystickButton(driver, Gamepad.BUTTON_B);
  // public static Button six = new JoystickButton(driver, Gamepad.BUTTON_A);
  // public static Button nine = new JoystickButton(driver, Gamepad.BUTTON_X);
  public static Button fuckass = new JoystickButton(driver, Gamepad.BUTTON_X);

  // Above all you've done is given a name to the button.
  // Below is where you will tell that button what to do.
  public OI(){
  // twelve.whenPressed(new SetClock(Clock.ClockPreset.TWELVE));
  // three.whenPressed(new SetClock(Clock.ClockPreset.THREE));
  // six.whenPressed(new SetClock(Clock.ClockPreset.SIX));
  // nine.whenPressed(new SetClock(Clock.ClockPreset.NINE));
  fuckass.whenPressed(new SetNeo(Neo.NeoPreset.MOVEIT));
  }
}
