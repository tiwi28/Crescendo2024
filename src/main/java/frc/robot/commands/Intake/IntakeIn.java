package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeIn extends Command {
  protected final Intake m_intake;

  public IntakeIn(Intake intake) {
    m_intake = intake;
  }

  @Override
  public void initialize() {
    m_intake.setSpeed(0.7); //Runs the intake motors at 70% power
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop(); //Stops the intake from running
  }

}
