package frc.robot.subsystems.Arm.Encoders;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmEncoderThroughbore implements ArmEncoder {
  private DutyCycleEncoder encoder;
  private Rotation2d offset = new Rotation2d();


  //Initialize encoder
  public ArmEncoderThroughbore(int id) {
    encoder = new DutyCycleEncoder(id);
  }

  //Sets the offset
  @Override
  public void setOffset(Rotation2d offset) {
    this.offset = offset;
  }

  //Calculates and returns the encoder's absolute position
  @Override
  public Rotation2d getAbsolutePosition() {
    return Rotation2d.fromDegrees(180 - (encoder.getAbsolutePosition() * 360 - offset.getDegrees()));
  }
}
