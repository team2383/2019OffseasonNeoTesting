package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.HAL;
import frc.robot.subsystems.Neo.NeoPreset;
import frc.robot.subsystems.Clock.ClockPreset;
import frc.robot.subsystems.Clock;

public class SetNeo extends Command {

    private NeoPreset neoPreset;

    DoubleSupplier neoManual;
    boolean manual;

    public SetNeo(NeoPreset neoPreset){
        requires(HAL.neo);
        manual = false;
        this.neoPreset = neoPreset;
    }

    public SetNeo(DoubleSupplier neoManual) {
      this.neoManual = neoManual;
      manual = true;
      requires(HAL.neo);
    }

    @Override
    protected void initialize() {
    }
  
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
      HAL.neo.setPosition(neoPreset);
    }
  
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
      return false;
    }
  
    // Called once after isFinished returns true
    @Override
    protected void end() {
    }
  
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}