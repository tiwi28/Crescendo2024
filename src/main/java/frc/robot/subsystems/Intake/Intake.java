package frc.robot.subsystems.Intake;


//Package imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IntakeConstants.IntakeMode;

public class Intake extends SubsystemBase {

  //Initializing intake motors
  private CANSparkMax leader = new CANSparkMax(41, MotorType.kBrushless);
  private CANSparkMax follower = new CANSparkMax(42, MotorType.kBrushless);

  private DigitalInput beamy = ShooterConstants.beam; //Beam break by the amp shooter
  public DigitalInput beamy2 = new DigitalInput(7); //Intake beam
  
  //Creates a new mode for intake when its not running(off)
  private IntakeMode intakeMode = IntakeMode.OFF;

  private double value = 0.7;

  public Intake() {
    //Sets the default settigs required to allow the subsystem to function effectively
    /*(**THIS INCLUDES SETTING VOLTAGE/CURRENT LIMITS AND CONTROLLING THE PARAMETERS*/
    leader.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(leader, Usage.kMinimal); //Minimizes
    leader.setSmartCurrentLimit(20, 20);
    leader.setIdleMode(IdleMode.kBrake);
    leader.enableVoltageCompensation(12);
    leader.setInverted(false);

    follower.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(follower, Usage.kMinimal);
    follower.setSmartCurrentLimit(20, 20);
    follower.setIdleMode(IdleMode.kBrake);
    follower.enableVoltageCompensation(12);
    leader.setInverted(false);

  }

  // makes sure that the motor controllers are configured properly
  public void burnToFlash() {
    leader.burnFlash();
    follower.burnFlash();
  }

  //Sets speed based on what the beam break sees
  public void setSpeed(double value) {
    if (beamy.get()) {
      leader.set(value);
      follower.set(value);
    } else {
      leader.set(0);
      follower.set(0);
    }
  }

  //Sets outtake speed
  public void setOutspeed(double value) {
    leader.set(value);
    follower.set(value);
  }

  //Sets the intake mode
  public void setIntakeMode(IntakeMode mode) {
    intakeMode = mode;
  }

  @Override
  public void periodic() {
    //@return true if an object is in between the beam break
    SmartDashboard.putBoolean("beamy2", beamy2.get());
  }

  //Stops the intake motors
  public void stop() {
    leader.set(0);
    follower.set(0);
  }

}
