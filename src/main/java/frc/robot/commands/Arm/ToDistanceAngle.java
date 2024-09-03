package frc.robot.commands.Arm;

import frc.robot.Constants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Drive.Swerve;

// This class extends ToAngle and is responsible for moving the robot's arm to a specific angle based on certain conditions.
public class ToDistanceAngle extends ToAngle {

  private Swerve m_drive; //Drive subsystem
  private ArmEndBehaviour m_endBehaviour; //The behavior of the arm's end state.

  //Defining the possible end behaviours of the arm
  public static enum ArmEndBehaviour {
    NORMAL,
    NEVER_ENDING
  }

  // Constructor that initializes the ToPoseAngle command with the drive and arm subsystems.
  // Sets the end behavior to NORMAL by default.
  public ToDistanceAngle(Swerve drive, Arm arm) {

    // Calls the parent constructor with a lambda function that predicts the target arm angle based on the robot's distance from a speaker.
    super(() -> {
      // var target =
      // Constants.ArmConstants.armAngleInterpolation.getTarget(drive.getDistanceFromAmp());

      // Uses a polynomial regression model to predict the target arm angle based on the robot's current position
      var target = Constants.ArmConstants.armAngleInterpolationPolynominalRegression
          .getPrediction(drive.getDistanceFromSpeaker());

      //Returns the predicted target angle for the arm
      return target;
    }, arm);

    m_drive = drive; //Initializes the drivesubsystem

    m_endBehaviour = ArmEndBehaviour.NORMAL;  //Sets the endBehavior to "NORMAL"
  }

  // Overloaded constructor that allows specifying a different end behavior.
  public ToDistanceAngle(Swerve drive, Arm arm, ArmEndBehaviour endBehaviour) {

    // Calls the parent constructor with a lambda function that predicts the target arm angle.
    super(() -> Constants.ArmConstants.armAngleInterpolationPolynominalRegression
        .getPrediction(drive.getDistanceFromSpeaker()), arm);
    m_drive = drive;  //Initializes the drive subsystem
    m_endBehaviour = endBehaviour;  //Sets the end behaviour to the speccified value
  }

  
  @Override
  public boolean isFinished() {
    //Determines if th ecommand is finished based on the current end behaviour
    switch (m_endBehaviour) {
      case NORMAL:
        return super.isFinished();  // Calls the parent class's isFinished method if the end behavior is NORMAL.
      case NEVER_ENDING:
        return false; // Always returns false, meaning the command never finishes if the end behavior is NEVER_ENDING.
      default:
        return super.isFinished(); // Default case returns the parent class's isFinished method.
    }
  }
}
