package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants.ClimberMode;
import frc.robot.subsystems.Climber.Climber;

//The base for all of the climber's functions 
public class ClimberBase extends Command {

  protected Climber climber; //Climber subsystem
  protected ClimberMode mode; //The climber modes

  public ClimberBase(Climber climber, ClimberMode mode) {
    this.climber = climber; //Initialize the climber subsystem
    this.mode = mode;

    this.addRequirements(climber);
  }

  @Override
  public void initialize() {
    //Sets climber mode
    climber.setMode(mode);
  }

  @Override
  //Returns code when the climber is done
  public boolean isFinished() {
    return climber.isDone();
  }

  @Override
  //Set the climbMode to "IDLE" when the command ends
  public void end(boolean interrupted) {
    climber.setMode(ClimberMode.IDLE);
  }
}
