package frc.robot.subsystems.Climber;

import java.util.Set;

//Imports
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberMode;

public class Climber extends SubsystemBase {
//Initializing the objects that allow us to control the robot's function

//Initializing motors
  private CANSparkMax leftClimberMotor = new CANSparkMax(Constants.ClimberConstants.leftMotorID,
      MotorType.kBrushless);
  private CANSparkMax rightClimberMotor = new CANSparkMax(Constants.ClimberConstants.rightMotorID,
      MotorType.kBrushless);

//Created a timer to ensure both climber arms would move uniformly
  private Timer timer = new Timer();

//PID Controllers
  private final SparkPIDController leftPID;
  private final SparkPIDController rightPID;

  //Encoders
  private final RelativeEncoder encoderLeft;
  private final RelativeEncoder encoderRight;

//Power
  private double leftPower = 0;
  private double rightPower = 0;

  //Climber Modes
  private ClimberMode climberMode = ClimberMode.IDLE;

  //Booleans that check if the left and right climber have stopped moving
  private boolean isLeftDone = false;
  private boolean isRightDone = false;

  public Climber() {
    setupMotors();

    //Creates an object for the encoder and PID controller of the left climber
    encoderLeft = leftClimberMotor.getEncoder();
    resetEncoderLeft(); //Sets encoder value to 0
    leftPID = leftClimberMotor.getPIDController();

    //Creates an object for the encoder and PID controller of the right climber
    encoderRight = rightClimberMotor.getEncoder();
    resetEncoderRight();//Sets encoder value to 0
    rightPID = rightClimberMotor.getPIDController();
  }
  

  //States that describe the climber arm's height
  private static enum States {
    BADBADBAD,
    OKAYUP,
    OKAYDOWN,
    GOOD
  }

  //Set the default settigs required to allow the subsystem to function effectively
    /*(**THIS INCLUDES SETTING VOLTAGE/CURRENT LIMITS)*/
  private void setupMotors() {
    leftClimberMotor.restoreFactoryDefaults();
    leftClimberMotor.enableVoltageCompensation(12);
    leftClimberMotor.setSmartCurrentLimit(20, 20);
    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    leftClimberMotor.setInverted(true);

    rightClimberMotor.restoreFactoryDefaults();
    rightClimberMotor.enableVoltageCompensation(12);
    rightClimberMotor.setSmartCurrentLimit(20, 20);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setInverted(true);
  }

  // makes sure that the motor controllers are configured properly
  public void burnToFlash() {
    leftClimberMotor.burnFlash();
    rightClimberMotor.burnFlash();
  }


  //Resets 
  public void setMode(ClimberMode mode) {
    climberMode = mode;
    if (climberMode == ClimberMode.HOMING) {
      timer.reset();
      timer.start();
    }
    isLeftDone = false;
    isRightDone = false;
  }

  //Sets the power going into the left/right motors
  public void set(double power) {
    setLeft(power);
    setRight(power);
  }

  public void stop() {
    //Stops left and right climber motors
    stopLeft();
    stopRight();
  }

//Sets the amount of power that goes into the left climber motor
  public void setLeft(double power) {
    leftPower = power;
  }

  //Sets the amount of power that goes into the right climber motor
  public void setRight(double power) {
    rightPower = power;
  }
//Stops left CLimber
  public void stopLeft() {
    setLeft(0);
  }

  //Stops right climber
  public void stopRight() {
    setRight(0);
  }

  //Resets encoder in left climber motor
  private void resetEncoderLeft() {
    encoderLeft.setPosition(0);
  }

  //Resets encoder in right cliber motor
  private void resetEncoderRight() {
    encoderRight.setPosition(0);
  }

  //Logs the encoder positions and current (in amps) on Glass
  public void logValues() {
    SmartDashboard.putNumber("Actual Climber Left", encoderLeft.getPosition());
    SmartDashboard.putNumber("Actual Climber Right", encoderRight.getPosition());
    
    SmartDashboard.putNumber("Climber Left Current", leftClimberMotor.getOutputCurrent());
    SmartDashboard.putNumber("Climber Right Current", rightClimberMotor.getOutputCurrent());
  }

//Checks to see if the climber is done moving.
  public boolean isDone() {
    return isLeftDone && isRightDone;
  }

//Checks the value of the encoder values and categorizes them into STATES
  public States outOfBounds(double encoderValue) {
    if (encoderValue <= ClimberConstants.min) {
      return States.BADBADBAD;
    } else if (encoderValue <= ClimberConstants.desiredMin) {
      return States.OKAYUP;
    } else if (encoderValue < ClimberConstants.desiredMax) {
      return States.GOOD;
    } else if (encoderValue < ClimberConstants.max) {
      return States.OKAYDOWN;
    } else if (encoderValue >= ClimberConstants.max) {
      return States.BADBADBAD;
    } else {
      return States.BADBADBAD;
    }
  }

  // Checks to see the position of right climb encoder
  public States isRightOutOfBounds() {
    return outOfBounds(-encoderRight.getPosition());
  }

  // Checks to see the position of right climb encoder
  public States isLeftOutOfBounds() {
    return outOfBounds(-encoderLeft.getPosition());
  }

  //Controls the movement of the left climber
  private void handleLeft() {
    switch (climberMode) {
      //Homes the climber
      case HOMING:
        if (leftClimberMotor.getOutputCurrent() >= ClimberConstants.homingCurrentThreshold) {
          if (timer.get() < 1)
            return;
          stopLeft();
          isLeftDone = true;
          encoderLeft.setPosition(0);
        } else {
          setLeft(ClimberConstants.selfHomeSpeedVoltage);
        }
        break;

      case DEPLOY:
        if (-encoderLeft.getPosition() >= ClimberConstants.max) {
          stopRight();
          isLeftDone = true;
        } else {
          setLeft(-ClimberConstants.deploySpeed);
        }
        break;

      case RETRACT:
        if (-encoderLeft.getPosition() <= ClimberConstants.min) {
          stopLeft();
          isLeftDone = true;
        } else {
          setLeft(ClimberConstants.retractSpeed);
        }
        break;

      case MANUAL:
        switch (isLeftOutOfBounds()) {
          case BADBADBAD:
            stopLeft();
            break;
          case OKAYUP:
            if (leftPower > 0) {
              setLeft(0);
            }
            break;
          case OKAYDOWN:
            if (leftPower < 0) {
              setLeft(0);
            }
            break;
          default:
            break;
        }
        break;

      case IDLE:
        stop();
        break;
      default:
        break;
    }
  }

  //Controls the movement of the right climber
  private void handleRight() {
    switch (climberMode) {
      //
      case HOMING:
        if (rightClimberMotor.getOutputCurrent() >= ClimberConstants.homingCurrentThreshold) {
          if (timer.get() < 1)
            return;
          stopRight();
          isRightDone = true;
          encoderRight.setPosition(0);
        } else {
          setRight(ClimberConstants.selfHomeSpeedVoltage);
        }
        break;
      case DEPLOY:
        if (encoderRight.getPosition() >= ClimberConstants.max) {
          stopRight();
          isRightDone = true;
        } else {
          setRight(-ClimberConstants.deploySpeed);
        }
        break;
      case RETRACT:
        if (encoderRight.getPosition() <= ClimberConstants.min) {
          stopRight();
          isRightDone = true;
        } else {
          setRight(ClimberConstants.retractSpeed);
        }
        break;
      case MANUAL:
        switch (isRightOutOfBounds()) {
          case BADBADBAD:
            stopRight();
            break;

          case OKAYUP:
            if (rightPower > 0) {
              setRight(0);
            }
            break;

          case OKAYDOWN:
            if (rightPower < 0) {
              setRight(0);
            }
            break;
          default:
            break;
        }
        break;

      
      case IDLE:
        stop();
        break;

      default:
        break;
    }
  }

  @Override
  public void periodic() {
    logValues();

  
    handleLeft();
    handleRight();

//Sets the controller reference value based on the selected control mode.

    if (leftPower > 1) {
      leftPID.setReference(leftPower, ControlType.kVoltage);
    } else {
      leftPID.setReference(leftPower, ControlType.kDutyCycle);
    }

    if (rightPower > 1) {
      rightPID.setReference(rightPower, ControlType.kVoltage);
    } else {
      rightPID.setReference(rightPower, ControlType.kDutyCycle);
    }
  }
}
