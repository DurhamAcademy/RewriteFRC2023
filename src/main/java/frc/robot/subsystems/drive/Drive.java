package frc.robot.subsystems.drive;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;

import java.io.IOException;

import static java.lang.Math.PI;

public class Drive extends SubsystemBase {
  private final ModuleIO[] moduleIOs = new ModuleIO[4];
  private final ModuleIOInputsAutoLogged[] inputs =
          new ModuleIOInputsAutoLogged[]{new ModuleIOInputsAutoLogged(),
                  new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged(),
                  new ModuleIOInputsAutoLogged()};

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final VisionIO visionIO;
  private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();
  private final CameraCamera alterCam = new CameraCamera();

  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3.0);
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
          new Translation2d(1.0, 1.0),
          new Translation2d(1.0, -1.0),
          new Translation2d(-1.0, 1.0),
          new Translation2d(-1.0, -1.0)
  );
  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics,
          new Rotation2d(),
          new SwerveModulePosition[]{
                  new SwerveModulePosition(0, new Rotation2d()),
                  new SwerveModulePosition(0, new Rotation2d()),
                  new SwerveModulePosition(0, new Rotation2d()),
                  new SwerveModulePosition(0, new Rotation2d())},
          new Pose2d());
  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
          AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField(),
          PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
          alterCam,
          new Transform3d(
                  new Translation3d(-0.258, 0.0, 0.137),
                  new Rotation3d(
                          0.0,
                          Units.degreesToRadians(-12.0),
                          Units.degreesToRadians(180.0))));

  boolean testMode = false;
  double testTime = 0.0;
  ProfiledPIDController anglePid;
  SimpleMotorFeedforward angleFF;
  double lastTime = Timer.getFPGATimestamp();

  /**
   * Creates a new Drive.
   *
   * @param flModuleIO
   * @param frModuleIO
   * @param blModuleIO
   * @param brModuleIO
   * @param visionIO
   */
  public Drive(ModuleIO flModuleIO,
               ModuleIO frModuleIO,
               ModuleIO blModuleIO,
               ModuleIO brModuleIO,
               GyroIO gyroIO, VisionIO visionIO) throws IOException {
    this(flModuleIO, frModuleIO, blModuleIO, brModuleIO, gyroIO, visionIO, false);
  }

  /** Creates a new Drive. */
  public Drive(ModuleIO flModuleIO,
               ModuleIO frModuleIO,
               ModuleIO blModuleIO,
               ModuleIO brModuleIO,
               GyroIO gyroIO,
               VisionIO visionIO,
               Boolean testMode) throws IOException {
    this.gyroIO = gyroIO;
    this.visionIO = visionIO;
    moduleIOs[0] = flModuleIO;
    moduleIOs[1] = frModuleIO;
    moduleIOs[2] = blModuleIO;
    moduleIOs[3] = brModuleIO;

    anglePid = new ProfiledPIDController(4.1487,
            0.0,
            .14002,
            new TrapezoidProfile.Constraints(PI * 4,
                    PI * 8));

    anglePid.enableContinuousInput(-Math.PI, Math.PI);

    angleFF = new SimpleMotorFeedforward(.25928, .28217, .0050137);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4; i++) {
      moduleIOs[i].updateInputs(inputs[i]);
      Logger.getInstance().processInputs(String.format("module [%d]", i), inputs[i]);
    }

    gyroIO.updateInputs(gyroInputs);
    Logger.getInstance().processInputs("gyro", gyroInputs);

    visionIO.updateInputs(visionInputs);
    Logger.getInstance().processInputs("camera", visionInputs);
    alterCam.setInputs(visionInputs);
    photonPoseEstimator.setReferencePose(getPose());
    photonPoseEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP);
    photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    var visionPose = photonPoseEstimator.update(visionInputs.cameraResult);
    visionPose.ifPresent(estimatedRobotPose ->
            estimatedRobotPose.targetsUsed.forEach(photonTrackedTarget ->
                    Logger.getInstance().recordOutput(
                            "Tag " + photonTrackedTarget.hashCode(),
                            new Pose3d(
                                    photonTrackedTarget.getBestCameraToTarget().getTranslation(),
                                    photonTrackedTarget.getBestCameraToTarget().getRotation()
                            )
                    )));
    visionPose.ifPresent(estimatedRobotPose -> poseEstimator.addVisionMeasurement(
            estimatedRobotPose.estimatedPose.toPose2d(),
            estimatedRobotPose.timestampSeconds
    ));
    // Update odometry and log the new pose
    var modulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      modulePositions[i] = new SwerveModulePosition(inputs[i].drivePositionRad * WHEEL_RADIUS_METERS, Rotation2d.fromRadians(inputs[i].turnAbsolutePositionRad));
      Logger.getInstance().processInputs(String.format("module [%d]", i), inputs[i]);
    }
    poseEstimator.updateWithTime(
            (testMode) ? testTime : Timer.getFPGATimestamp(),
            Rotation2d.fromDegrees(gyroInputs.yaw),
            modulePositions
    );
    Logger.getInstance().recordOutput("Pose Estimator", getPose());
    Logger.getInstance().recordOutput("SwerveModule " + 0,
            new SwerveModuleState(
                    inputs[0].driveVelocityRadPerSec * WHEEL_RADIUS_METERS,
                    Rotation2d.fromRadians(inputs[0].turnAbsolutePositionRad)),
            new SwerveModuleState(
                    inputs[1].driveVelocityRadPerSec * WHEEL_RADIUS_METERS,
                    Rotation2d.fromRadians(inputs[1].turnAbsolutePositionRad)),
            new SwerveModuleState(
                    inputs[2].driveVelocityRadPerSec * WHEEL_RADIUS_METERS,
                    Rotation2d.fromRadians(inputs[2].turnAbsolutePositionRad)),
            new SwerveModuleState(
                    inputs[3].driveVelocityRadPerSec * WHEEL_RADIUS_METERS,
                    Rotation2d.fromRadians(inputs[3].turnAbsolutePositionRad)));
  }


  /** Run open loop based on stick positions. */
  public void driveArcade(double xSpeed, double ySpeed, double zRotation, boolean fieldRelative) {
    var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(xSpeed * 12, ySpeed * 12, 12 * zRotation),//.slewLimited(xSlewRateLimiter, ySlewRateLimiter, rotSlewRateLimiter),
            getPose().getRotation()
    );
    var speeds = kinematics.toSwerveModuleStates(chassisSpeeds);
    for (int i = 0; i < speeds.length; i++) {
      var speed = speeds[i];
      moduleIOs[i].setDriveVoltage(speed.speedMetersPerSecond);
      var rad = speed.angle.getRadians();

      var lastVel = anglePid.getSetpoint().velocity;
      var t = Timer.getFPGATimestamp();
      var dt = t - lastTime;
      var pid = anglePid.calculate(inputs[i].turnAbsolutePositionRad, rad);
      var anglePower = -(pid) + angleFF.calculate(
              lastVel,
              anglePid.getSetpoint().velocity,
              Constants.loopPeriodSecs//minOf(dt,0.001) // stop possible divide by zero?
      );
      moduleIOs[i].setTurnVoltage(anglePower);
    }

    Logger.getInstance().recordOutput("Swerve Intensions", speeds);
  }

  /** Stops the drive. */
  public void stop() {
    for (ModuleIO moduleIO : moduleIOs) {
      moduleIO.setTurnVoltage(0.0);
      moduleIO.setDriveVoltage(0.0);
      moduleIO.setTurnBrakeMode(true);
      moduleIO.setDriveBrakeMode(true);
    }
  }

  /** Returns the current odometry pose in meters. */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }
}
