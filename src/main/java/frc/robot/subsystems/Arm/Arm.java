
//Imports
package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.Encoders.ArmEncoder;
import frc.robot.subsystems.Arm.Encoders.ArmEncoderThroughbore;

public class Arm extends SubsystemBase {
  //Initialize motors
  private final CANSparkMax leader = new CANSparkMax(Constants.ArmConstants.leaderID, MotorType.kBrushless);
  private final CANSparkMax follower = new CANSparkMax(Constants.ArmConstants.followerID, MotorType.kBrushless);

  //Initialize encoders
  private final ArmEncoder encoder = new ArmEncoderThroughbore(Constants.ArmConstants.encoderID);
  public Rotation2d encoderPosition = new Rotation2d();

  //PID controller + feedforward initialization
  private final PIDController pid = new PIDController(Constants.ArmConstants.armPID[0],
      Constants.ArmConstants.armPID[1],
      Constants.ArmConstants.armPID[2]);
  private ArmFeedforward ffModel = new ArmFeedforward(
      Constants.ArmConstants.armSGV[0],
      Constants.ArmConstants.armSGV[1],
      Constants.ArmConstants.armSGV[2]);

  //Motion profiling 
  private Rotation2d setpoint = new Rotation2d();
  private Rotation2d velocity = new Rotation2d();
  private Rotation2d goal = new Rotation2d();

  // Tunable values (Values can be changed using Glass)
  private LoggedTunableNumber armP = new LoggedTunableNumber("armP", Constants.ArmConstants.armPID[0]);
  private LoggedTunableNumber armI = new LoggedTunableNumber("armI", Constants.ArmConstants.armPID[1]);
  private LoggedTunableNumber armD = new LoggedTunableNumber("armD", Constants.ArmConstants.armPID[2]);
  private LoggedTunableNumber armS = new LoggedTunableNumber("armS", Constants.ArmConstants.armSGV[0]);
  private LoggedTunableNumber armG = new LoggedTunableNumber("armG", Constants.ArmConstants.armSGV[1]);
  private LoggedTunableNumber armV = new LoggedTunableNumber("armV", Constants.ArmConstants.armSGV[2]);


  public Arm() {
    setupMotors();
    //Set offset + get encoders position
    encoder.setOffset(Constants.ArmConstants.offset);
    encoderPosition = encoder.getAbsolutePosition();
    runSetpoint(getEncoderPosition());
  }

  //Makes sure that the motor controllers are configured properly
  public void burnToFlash() {
    leader.burnFlash();
    follower.burnFlash();
  }

  //Motor Set-Up
  private void setupMotors() {
    leader.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(leader, Usage.kPositionOnly);
    leader.enableVoltageCompensation(12);
    leader.setSmartCurrentLimit(30, 30);
    leader.setIdleMode(IdleMode.kBrake);
    leader.setInverted(false);

    follower.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(follower, Usage.kPositionOnly);
    follower.enableVoltageCompensation(12);
    follower.setSmartCurrentLimit(30, 30);
    follower.setIdleMode(IdleMode.kBrake);
    follower.setInverted(true);

    pid.setIntegratorRange(-0.05, 0.05);
  }

  //Resets the PID's i value 
  public void resetI() {
    pid.reset();
  }

  //returns the encoder's position
  public Rotation2d getEncoderPosition() {
    return encoderPosition;
  }


  //Checks to see if the PID and SGV values have changed. 
  //Updates the value if it's been changed
  public void checkTunableValues() {
    if (!Constants.enableTunableValues)
      return;

    if (armP.hasChanged() || armI.hasChanged() || armD.hasChanged()) {
      pid.setPID(armP.get(), armI.get(), armD.get());
    }

    if (armS.hasChanged() || armG.hasChanged() || armV.hasChanged()) {
      ffModel = new ArmFeedforward(armS.get(), armG.get(), armV.get());
    }
  }

  //Sets 
  public void setGoal(Rotation2d goal) {
    this.goal = goal;
  }
//Check if arm is at goal
  public boolean atGoal() {
    return Math.abs(getEncoderPosition().getRadians() - goal.getRadians()) < Constants.ArmConstants.tolernace
        .getRadians();
  }

  //Ensures the arm remains within its minimum and maximum angle values
  public void runSetpoint(Rotation2d setpoint) {
    if (setpoint.getRadians() < Constants.ArmConstants.min.getRadians()) {
      this.setpoint = Constants.ArmConstants.min;
    } else if (setpoint.getRadians() > Constants.ArmConstants.max.getRadians()) {
      this.setpoint = Constants.ArmConstants.max;
    } else {
      this.setpoint = setpoint;
    }
  }

//Set arm velocity
  public void runVelocity(Rotation2d velocity) {
    this.velocity = velocity;
  }

  //Update the profile's state
  public void runState(TrapezoidProfile.State state) {
    runSetpoint(Rotation2d.fromRadians(state.position));
    runVelocity(Rotation2d.fromRadians(state.velocity));
  }

  //Get the current trapezoi
  public TrapezoidProfile.State getCurrenState() {
    return new TrapezoidProfile.State(encoderPosition.getRadians(), this.velocity.getRadians());
  }

  //Checks if arm is within tolerance
  public boolean atSetpoint() {
    return Math.abs(getEncoderPosition().getRadians() - setpoint.getRadians()) < Constants.ArmConstants.tolernace
        .getRadians();
  }

  //Get setpoint
  public Rotation2d getSetpoint() {
    return setpoint;
  }

  //Get velocity
  public Rotation2d getVelovity() {
    return velocity;
  }

  //Logs values to SmartDashboard/Glass
  private void logValues() {
    SmartDashboard.putNumber("Arm Actual Angle", getEncoderPosition().getDegrees());
    SmartDashboard.putNumber("Arm Desired Angle", setpoint.getDegrees());
    SmartDashboard.putNumber("Arm Desired Speed", velocity.getDegrees());
  }

  @Override
  public void periodic() {

    encoderPosition = encoder.getAbsolutePosition();  //Finds the exact position of the encoder
    logValues();  //Logs values to SmartDashboard/Glass
    checkTunableValues(); //Updates PID and FF values


    var ffOutput = ffModel.calculate(setpoint.getRadians(), velocity.getRadians()); //Calculates Feedforward output
    var pidOutput = pid.calculate(getEncoderPosition().getRadians(), setpoint.getRadians());  //calculates PID output

    SmartDashboard.putNumber("ffoutput arm", ffOutput); //Displays the FF output calculated above on Smartdahsboard/Glass

    //PID+FF output on the leader and follower motors
    leader.set(ffOutput + pidOutput);
    follower.set(ffOutput + pidOutput);
  }

}
