package frc.robot.subsystems.Drive;

//Imports
import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.SwerveConstants.DriveMode;

public class Swerve extends SubsystemBase {
  private SwerveModule[] mSwerveMods;
  private GyroPigeon2 gyro;

  private final SwerveDrivePoseEstimator poseEstimator;

  public final VisionPhotonvision vision = new VisionPhotonvision();

  private final PIDController snapPIDController;
  private SimpleMotorFeedforward snapFFModel;
  private DriveMode driveMode = DriveMode.DriverInput;
  private Rotation2d snapSetpoint = new Rotation2d();
  private Rotation2d snapVelocity = new Rotation2d();
  private Rotation2d snapGoal = new Rotation2d();

  private Field2d debugField2d = new Field2d();
  private Pose2d lastActivePathPose = new Pose2d();

  private Translation2d robotSpeed = new Translation2d();

  // Tunable values
  private LoggedTunableNumber driveP = new LoggedTunableNumber("driveP", Constants.SwerveConstants.drivePID[0]);
  private LoggedTunableNumber driveI = new LoggedTunableNumber("driveI", Constants.SwerveConstants.drivePID[1]);
  private LoggedTunableNumber driveD = new LoggedTunableNumber("driveD", Constants.SwerveConstants.drivePID[2]);

  private LoggedTunableNumber driveS = new LoggedTunableNumber("driveS", Constants.SwerveConstants.driveSVA[0]);
  private LoggedTunableNumber driveV = new LoggedTunableNumber("driveV", Constants.SwerveConstants.driveSVA[1]);
  private LoggedTunableNumber driveA = new LoggedTunableNumber("driveA", Constants.SwerveConstants.driveSVA[2]);

  private LoggedTunableNumber angleP = new LoggedTunableNumber("angleP", Constants.SwerveConstants.anglePID[0]);
  private LoggedTunableNumber angleI = new LoggedTunableNumber("angleI", Constants.SwerveConstants.anglePID[1]);
  private LoggedTunableNumber angleD = new LoggedTunableNumber("angleD", Constants.SwerveConstants.anglePID[2]);

  private LoggedTunableNumber snapP = new LoggedTunableNumber("SnapP", Constants.SwerveConstants.snapPID[0]);
  private LoggedTunableNumber snapI = new LoggedTunableNumber("SnapI", Constants.SwerveConstants.snapPID[1]);
  private LoggedTunableNumber snapD = new LoggedTunableNumber("SnapD", Constants.SwerveConstants.snapPID[2]);

  private LoggedTunableNumber snapS = new LoggedTunableNumber("SnapS", Constants.SwerveConstants.snapSVA[0]);
  private LoggedTunableNumber snapV = new LoggedTunableNumber("SnapV", Constants.SwerveConstants.snapSVA[1]);
  private LoggedTunableNumber snapA = new LoggedTunableNumber("SnapA", Constants.SwerveConstants.snapSVA[2]);

  public Swerve() {
    /* Gyro setup */
    gyro = new GyroPigeon2(Constants.SwerveConstants.pigeonID);
    gyro.home();

    /* Swerve modules setup */
    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
        new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
        new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
        new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
    };
    
    //Pose estimator
    poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions(),
        new Pose2d());

    //PID values for the drivetrain's 'snapping' function. 
    snapPIDController = new PIDController(
        Constants.SwerveConstants.snapPID[0],
        Constants.SwerveConstants.snapPID[1],
        Constants.SwerveConstants.snapPID[2]);

    //Feedforward
    snapFFModel = new SimpleMotorFeedforward(
        Constants.SwerveConstants.snapSVA[0],
        Constants.SwerveConstants.snapSVA[1],
        Constants.SwerveConstants.snapSVA[2]);
    
    // Set tolerance for snapPIDController in radians (converted from degrees).
    snapPIDController.setTolerance(Units.degreesToRadians(5));
    

  // Enable continuous input for snapPIDController between 0 and 2π radians.
  //Causes the PID contriller to take the shortest path while rotating
    snapPIDController.enableContinuousInput(0, Math.PI * 2);
    
    // Set the integrator range for snapPIDController to limit integral windup.
    snapPIDController.setIntegratorRange(-0.1, 0.1);

    // Configure the holonomic path follower for autonomous driving.
    AutoBuilder.configureHolonomic(this::getEstimatedPose, this::setPose, this::getRobotRelativeSpeeds,
        this::driveRobotRelative, new HolonomicPathFollowerConfig(
            Constants.AutoConstants.translationPID, // Translation constants
            Constants.AutoConstants.rotationPID, // Rotation constants
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.SwerveConstants.wheelBase / 2, // Drive base radius (distance from center to furthest module)
            new ReplanningConfig()),

        //Determines if the alliance is red
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, this);

    //Set a callback to log the target pose in pathplanner
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          debugField2d.getObject("pathplanner target pose").setPose(targetPose);
        });
    
    // Set a callback to log the active path in PathPlanner.
    PathPlannerLogging.setLogActivePathCallback(this::handleActivePathLogger);
    
    // Add the vision field to the SmartDashboard for visualization.
    SmartDashboard.putData("Vision field", debugField2d);

    Timer.delay(4); // Let the can bus cool down
  }

  // Handle active path logging by storing the last active pose.
  private void handleActivePathLogger(List<Pose2d> poses) {
    if (poses.isEmpty())
      return;

    lastActivePathPose = poses.get(poses.size() - 1);
  }

  // Get the last active path pose.
  private Pose2d getLastActivePathPose() {
    return lastActivePathPose;
  }


  // Retrieve the snap target override if in Snap mode.
  public Optional<Rotation2d> getRotationTargetOverride() {
    if (driveMode == DriveMode.Snap) {
      return Optional.of(snapGoal);
    } else {
      return Optional.empty();
    }
  }

  //Set the snap setpoint and velocity
  public void setSnapSetpoint(Rotation2d setpoint, Rotation2d velocity) {
    this.snapSetpoint = setpoint;
    this.snapVelocity = velocity;
  }

  //Set the snap goal
  public void setSnapGoal(Rotation2d goal) {
    this.snapGoal = goal;
  }

  //Checks if the snap is at goal (or within the tolerance range)
  public boolean isSnapAtGoal() {
    double errorBound = (Math.PI * 2) / 2.0;
    var m_positionError = MathUtil.inputModulus(snapGoal.getRadians() - getYawForSnap().getRadians(), -errorBound,
        errorBound);
    return Math.abs(m_positionError) < Units.degreesToRadians(5);
  }

  //Checks if snap is at the setpoint
  public boolean isSnapAtSetpoint() {
    return snapPIDController.atSetpoint();
  }

  //Gets the robot's yaw for snap
  public Rotation2d getYawForSnap() {
    double yawRad = getYaw().getRadians();
    yawRad = yawRad % (Math.PI * 2);
    if (yawRad < 0) {
      yawRad = Math.PI * 2 + yawRad;
    }

    return new Rotation2d(yawRad);
  }

  //Calculate the output for the snap using PID and feedforward control
  public double calculateSnapOutput(Rotation2d setpoint, Rotation2d snapVelocity) {
    double ffOutput = -snapFFModel.calculate(snapVelocity.getRadians());
    double rotation = -snapPIDController.calculate(getYawForSnap().getRadians(), snapSetpoint.getRadians());
    rotation = MathUtil.clamp(rotation, -1, 1);
    rotation /= 2;
    return rotation + ffOutput;
  }

  /**
   * The main function used for driving the robot
   * 
   * @param translation
   * @param rotation
   */

  
  public void drive(Translation2d translation, double rotation) {
    switch (driveMode) {
      case Snap:
        rotation = calculateSnapOutput(snapSetpoint, snapVelocity);
        break;

      case AutonomousSnap:
        rotation = calculateSnapOutput(snapSetpoint, snapVelocity);
        break;

      default:
        break;
    }

    // Calculate the swerve module states for robot movement based on translation and rotation.
    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX() * Constants.SwerveConstants.maxSpeed,
            translation.getY() * Constants.SwerveConstants.maxSpeed,
            rotation * Constants.SwerveConstants.maxAngularVelocity, getYaw()));
    
    //The robot's speed
    robotSpeed = new Translation2d(translation.getX() * Constants.SwerveConstants.maxSpeed,
        translation.getY() * Constants.SwerveConstants.maxSpeed);
    
    //Setting the max speed to ensure they don't exceed the maximum speed limit
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);
    
    // Set the desired states for the swerve modules.
    setStates(swerveModuleStates);
  }
  
  // Drive the robot relative to its own frame of reference.
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    double rotation = targetSpeeds.omegaRadiansPerSecond;
    targetSpeeds = new ChassisSpeeds(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond,
        -rotation);

    var currentSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds.vxMetersPerSecond,
        robotRelativeSpeeds.vyMetersPerSecond,
        robotRelativeSpeeds.omegaRadiansPerSecond, poseEstimator.getEstimatedPosition().getRotation());
    robotSpeed = new Translation2d(currentSpeed.vxMetersPerSecond, currentSpeed.vyMetersPerSecond);

    SwerveModuleState[] targetStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  // Set the desired states for the swerve modules.
  public void setStates(SwerveModuleState[] states) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(states[mod.moduleNumber]);
    }
  }

  //Sets drive mode
  public void setDriveMode(DriveMode mode) {
    driveMode = mode;
  }

  //Gets the current drive mode
  public DriveMode getDriveMode() {
    return driveMode;
  }

  //Set the desired states for the swerve modules
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber]);
    }
  }

  //Resets the Snap PIDController
  public void resetSnapI() {
    snapPIDController.reset();
  }

  /** Reset the module encoder values */
  public void resetModuleZeros() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  /**
   * Get the current state of the modules
   * 
   * @return state of the modules
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  //Get the position of each swerve module
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }

    return positions;
  }

    //Get the robot's relative speeds from the swerve modules' states
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getStates());
  }

  /**
   * get the orientation of the robot
   * 
   * @return the orientation of the robot
   */
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  /**
   * Get the pitch from gyro
   * 
   * @return the pitch of the robot
   */
  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(gyro.getPitch());
  }

    /**
   * Get the roll from gyro
   * 
   * @return the roll of the robot
   */
  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(gyro.getRoll());
  }

  
  public void zeroGyro() {
    gyro.homeYaw();
  }

   //Updates PID and Feedforward values if the number has changed
  public void checkTunableValues() {
    if (!Constants.enableTunableValues)
      return;

    //Updates the anglePID values if the P, I, or D value changes
    if (angleP.hasChanged() || angleI.hasChanged() || angleD.hasChanged()) {
      for (SwerveModule mod : mSwerveMods) {
        mod.angleController.setP(angleP.get());
        mod.angleController.setI(angleI.get());
        mod.angleController.setD(angleD.get());
      }
    }
    //Updates the drivePID values if the P, I, or D value changes
    if (driveP.hasChanged() || driveI.hasChanged() || driveD.hasChanged()) {
      for (SwerveModule mod : mSwerveMods) {
        mod.driveController.setP(driveP.get());
        mod.driveController.setI(driveI.get());
        mod.driveController.setD(driveD.get());

        // mod.drivePIDController.setPID(driveP.get(), driveI.get(), driveD.get());
      }
    }

    //Updates the driveSVA values if the P, I, or D value changes
    if (driveS.hasChanged() || driveV.hasChanged() || driveA.hasChanged()) {
      for (SwerveModule mod : mSwerveMods) {
        mod.feedforward = new SimpleMotorFeedforward(driveS.get(), driveV.get(), driveA.get());
      }
    }
    //Updates the snapPID values if the P, I, or D value changes
    if (snapP.hasChanged() || snapI.hasChanged() || snapD.hasChanged()) {
      snapPIDController.setPID(snapP.get(), snapI.get(), snapD.get());
    }

    //Updates the snapSVA values if the S, V, or A value changes
    if (snapS.hasChanged() || snapV.hasChanged() || snapA.hasChanged()) {
      snapFFModel = new SimpleMotorFeedforward(snapS.get(), snapV.get(), snapA.get());
    }
  }

  //Updats vision measurements
  public void updateVisionMeasurements() {
    var visionEst = vision.getEstimatedGlobalPose();
    visionEst.ifPresent(est -> {
      if (est.estimatedPose.toPose2d().getX() > 4) {
        return;
      }
      var estPose = est.estimatedPose.toPose2d();
      var estStdDevs = vision.getEstimationStdDevs(estPose);
      poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    });
  }

  //Get the Pose from the speaker
  private Optional<Pose3d> getSpeakerPose() {
    var alliance = DriverStation.getAlliance();
    Optional<Pose3d> ampPose = null;

    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Blue) {
        ampPose = vision.kTagLayout.getTagPose(7);
      } else {
        ampPose = vision.kTagLayout.getTagPose(4);
      }
    } else {
      ampPose = vision.kTagLayout.getTagPose(7);
    }

    return ampPose;
  }

  //Get the robot's estimated position
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  //Resets the Pose and updates it
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getYaw(), getPositions(), pose);
    vision.resetPose(pose);
  }

  //Get distance from speaker during autos
  public double getAutoPoseDistanceFromSpeaker() {
    return lastActivePathPose.getTranslation().getDistance(getSpeakerPose().get().getTranslation().toTranslation2d());
  }
//Get distance from speaker
  public double getDistanceFromSpeaker() {
    return getEstimatedPose().getTranslation().getDistance(getSpeakerPose().get().getTranslation().toTranslation2d());
  }

  //Get the robot's rotation relative to the speaker 
  public Rotation2d getRotationRelativeToSpeaker() {
    var alliance = DriverStation.getAlliance();

    if (!alliance.isPresent())
      return new Rotation2d();

    switch (alliance.get()) {
      case Blue:
        return new Rotation2d(
            getEstimatedPose().getTranslation().minus(getSpeakerPose().get().getTranslation().toTranslation2d())
                .unaryMinus()
                .getAngle().getRadians() + Math.PI);
      case Red:
        return getEstimatedPose().getTranslation().minus(getSpeakerPose().get().getTranslation().toTranslation2d())
            .unaryMinus()
            .getAngle();

      default:
        return new Rotation2d();
    }
  }

  //Gets the closest tag
  public Optional<PhotonTrackedTarget> getClosestTag() {
    var targets = vision.getLatestResult().getTargets();
    Optional<PhotonTrackedTarget> result = Optional.empty();

    for (var target : targets) {
      if (target.getPoseAmbiguity() > 0.3) {
        break;
      }
      if (result.isPresent()) {
        if (result.get().getBestCameraToTarget().getTranslation().getNorm() > target.getBestCameraToTarget()
            .getTranslation().getNorm())
          result = Optional.of(target);
      } else {
        result = Optional.of(target);
      }
    }

    return result;
  }

  //Values we recorded, can be viewed on Glass
  public void logValues() {
    SmartDashboard.putNumber("Distance From Speaker", getDistanceFromSpeaker());
    SmartDashboard.putNumber("Rotation to Speaker", getRotationRelativeToSpeaker().getDegrees());

    debugField2d.setRobotPose(getEstimatedPose());
    if (vision.latestVision.isPresent()) {
      debugField2d.getObject("Vision Intrusive Thoughts").setPose(vision.latestVision.get().estimatedPose.toPose2d());
    }

    SmartDashboard.putNumber("Snap Setpoint", snapSetpoint.getDegrees());
    SmartDashboard.putNumber("Snap Actual", getYawForSnap().getDegrees());

  }

  //Makes sure motor controllers are configured properly
  public void burnToFlash() {
    for (SwerveModule mod : mSwerveMods) {
      mod.burnToFlash();
    }
  }

  //Get the speed compensation angle
  public Rotation2d getSpeedCompensationAngle() {
    return Rotation2d.fromDegrees(
        (robotSpeed.getY() * Constants.ShooterConstants.onTheFlyMultiplier)
            * ((FieldConstants.fieldLength - getDistanceFromSpeaker()) / FieldConstants.fieldLength));
  }

  @Override
  public void periodic() {

    updateVisionMeasurements(); //Updates vision measurements
    checkTunableValues(); 



    /*Ensures each swerve module has its values logged  
    In this case, the desired/actual speeds and angles for each module will be recorded
    and can be viewed on SmartDashboard or Glass*/
    for (SwerveModule mod : mSwerveMods) {
      mod.logValues();
    }
    gyro.logValues();

    poseEstimator.update(getYaw(), getPositions());

    switch (driveMode) {
      case AutonomousSnap:
        drive(new Translation2d(), 0);
        break;
      default:
        break;
    }

    for (SwerveModule mod : mSwerveMods) {
      mod.periodic();
    }
    logValues();
  }
}
