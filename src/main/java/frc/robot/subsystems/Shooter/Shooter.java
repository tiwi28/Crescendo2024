package frc.robot.subsystems.Shooter;


// Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  //Initialized motor controllers, encoders, pid controllers
  private final CANSparkMax leader = new CANSparkMax(Constants.ShooterConstants.leader, MotorType.kBrushless);
  private final CANSparkMax follower = new CANSparkMax(Constants.ShooterConstants.follower, MotorType.kBrushless);

  private final RelativeEncoder encoder = leader.getEncoder();
  // private final RelativeEncoder Rencoder = follower.getEncoder();
  private final SparkPIDController pid = leader.getPIDController();
  // private final SparkPIDController Rpid = follower.getPIDController();
  // Uncomment all of Rpid to split the shooter in the code

  //Shooter Feed-Forward
  private SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(Constants.ShooterConstants.shooterFeedforward[0],
      Constants.ShooterConstants.shooterFeedforward[1]);
  // private SimpleMotorFeedforward RffModel = new
  // SimpleMotorFeedforward(Constants.ShooterConstants.shooterFeedforward[0],
  // Constants.ShooterConstants.shooterFeedforward[1]);
  private double velocitySetpoint = 0;
  private double velocityRateOfChange = 0;

  //PID Values
  private LoggedTunableNumber shooterKp = new LoggedTunableNumber("shooterKp",
      Constants.ShooterConstants.shooterPID[0]);
  private LoggedTunableNumber shooterKi = new LoggedTunableNumber("shooterKi",
      Constants.ShooterConstants.shooterPID[1]);
  private LoggedTunableNumber shooterKd = new LoggedTunableNumber("shooterKd",
      Constants.ShooterConstants.shooterPID[2]);

  //Static and Velocity gain 
  private LoggedTunableNumber shooterKs = new LoggedTunableNumber("shooterKs",
      Constants.ShooterConstants.shooterFeedforward[0]);
  private LoggedTunableNumber shooterKv = new LoggedTunableNumber("shooterKv",
      Constants.ShooterConstants.shooterFeedforward[1]);

  public Shooter() {
    setupMotor();
  }

  //Motor settings
  public void setupMotor() {
    leader.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(leader, Usage.kVelocityOnly);
    leader.setInverted(true);
    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(60, 60);
    leader.setInverted(true);

    follower.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(follower, Usage.kVelocityOnly);
    follower.setInverted(true);
    follower.enableVoltageCompensation(12.0);
    follower.setSmartCurrentLimit(60, 60);
    follower.setInverted(true);

    follower.follow(leader, true);

    encoder.setVelocityConversionFactor(1);

    //Setting Shooter PID values
    pid.setP(Constants.ShooterConstants.shooterPID[0]);
    pid.setI(Constants.ShooterConstants.shooterPID[1]);
    pid.setD(Constants.ShooterConstants.shooterPID[2]);
    // Rpid.setP(Constants.ShooterConstants.shooterPID[0]);
    // Rpid.setI(Constants.ShooterConstants.shooterPID[1]);
    // Rpid.setD(Constants.ShooterConstants.shooterPID[2]);

    //Limits the I-value to help manage integral wind-up 
    pid.setIMaxAccum(0.04, 0);
    // Rpid.setIMaxAccum(0.04, 0);

    //Sets the range limit for power output
    pid.setOutputRange(0, 9999);
    // Rpid.setOutputRange(0, 9999);
  }

  //Makes sure that the motor controllers are configured properly
  public void burnToFlash() {
    follower.burnFlash();
    leader.burnFlash();
  }

  //Resets the i value
  public void resetI() {
    pid.setIAccum(0);
    // Rpid.setIAccum(0);
  }



  public void checkTunableValues() {
    if (!Constants.enableTunableValues)
      return;

    //Changes the PID values to the set tunable value when its changed
    if (shooterKp.hasChanged() || shooterKi.hasChanged() || shooterKd.hasChanged()) {
      pid.setP(shooterKp.get());
      pid.setI(shooterKi.get());
      pid.setD(shooterKd.get());
      // Rpid.setP(shooterKp.get());
      // Rpid.setI(shooterKi.get());
      // Rpid.setD(shooterKd.get());
    }

    //Updates FF when the s or v values are changed
    if (shooterKs.hasChanged() || shooterKv.hasChanged()) {
      ffModel = new SimpleMotorFeedforward(shooterKs.get(), shooterKv.get());
      // Rffmodel = new SimpleMotorFeedforward(shooterKs.get(), shooterKv.get());
    }
  }

//Sets the velocity and acceletation
  public void runVelocity(double rpm, double rpmPerSecond) { // Add rpmR
    this.velocitySetpoint = rpm;
    this.velocityRateOfChange = rpmPerSecond;
    // this.RvelocitySetpoint = rpmR;
  }

  //Stops Shooter Motor
  public void stop() {
    runVelocity(0, 0);
  }
 
  public double getActualRPMleader() {
    return encoder.getVelocity();
  }
  // public double getActualRPMFollower() {
  // return Rencoder.getVelocity();
  // }


  public double getDesiredRPMleader() {
    return velocitySetpoint;
  }
  // public double getDesiredRPMfollower() {
  // return RvelocitySetpoint;
  // }

  //Gets the acceleration (m/s^2)
  public double getDesiredRPMPerSecond() {
    return velocityRateOfChange;
  }

  //Checks to see if the actual velocity is within tolerance
  public boolean atSetpoint() {
    return Math.abs(getActualRPMleader() - velocitySetpoint) < Constants.ShooterConstants.toleranceRPM;
  }


  // public boolean atRsetpoint() {
  // return Math.abs(getActualRPMFollower() - velocitySetpoint <
  // Constants.ShooterConstants.toleranceRPM);
  // }

  //Log values for Glass
public void logValues() {
    SmartDashboard.putNumber("Shooter Actual RPM", getActualRPMleader());
    // SmartDashboard.putNumber("Shooter Actual RPM", getActualRPMFollower());

    SmartDashboard.putNumber("Shooter Desired RPM", getDesiredRPMleader());
    SmartDashboard.putNumber("Shooter Desired RPM/s", velocityRateOfChange);

  }

  @Override
  public void periodic() {

    //Feedforward calculations
    double feedforward = ffModel.calculate(velocitySetpoint, velocityRateOfChange);

    //Sets reference for the PID controller
    pid.setReference(velocitySetpoint, ControlType.kVelocity, 0, feedforward);


    // Rpid.setReference(velocitySetpoint, ControlType.kVelocity, 0, feedforward);
    // double Rfeedforward = Rffmodel.calculate(RvelocitySetpoint,
    // velocityRateOfChange);
    checkTunableValues();
    logValues();
  }
}
