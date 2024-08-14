package frc.robot.subsystems.Arm.Encoders;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Interface for an encoder used in the robot arm subsystem.
 * The encoder provides the absolute position of the arm and allows setting an offset.
 */

public interface ArmEncoder {

  //@return the exact position of the arm encoder
  public Rotation2d getAbsolutePosition();

  //Sets the arm's offset
  public void setOffset(Rotation2d offset);
}
