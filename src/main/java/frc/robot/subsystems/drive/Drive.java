package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static java.lang.Math.PI;

public class Drive extends SubsystemBase {
  private final ModuleIO[] moduleIOs = new ModuleIO[4];
  private final ModuleIOInputsAutoLogged[] inputs =
          new ModuleIOInputsAutoLogged[]{new ModuleIOInputsAutoLogged(),
                  new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged(),
                  new ModuleIOInputsAutoLogged()};

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
   */
  public Drive(ModuleIO flModuleIO,
               ModuleIO frModuleIO,
               ModuleIO blModuleIO,
               ModuleIO brModuleIO) {
    this(flModuleIO, frModuleIO, blModuleIO, brModuleIO, false);
  }

  /** Creates a new Drive. */
  public Drive(ModuleIO flModuleIO,
               ModuleIO frModuleIO,
               ModuleIO blModuleIO,
               ModuleIO brModuleIO,
               Boolean testMode) {
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


    // Update odometry and log the new pose
    var modulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      modulePositions[i] = new SwerveModulePosition(inputs[i].drivePositionRad * WHEEL_RADIUS_METERS, Rotation2d.fromRadians(inputs[i].turnAbsolutePositionRad));
      Logger.getInstance().processInputs(String.format("module [%d]", i), inputs[i]);
    }
    poseEstimator.updateWithTime(
            (testMode) ? testTime : Timer.getFPGATimestamp(),
            new Rotation2d(),
            modulePositions
    );
    Logger.getInstance().recordOutput("Pose Estimator", poseEstimator.getEstimatedPosition());
    Logger.getInstance().recordOutput("SwerveModule " + 0,
            new SwerveModuleState(
                    inputs[0].drivePositionRad * WHEEL_RADIUS_METERS,
                    Rotation2d.fromRadians(inputs[0].turnAbsolutePositionRad)),
            new SwerveModuleState(
                    inputs[1].drivePositionRad * WHEEL_RADIUS_METERS,
                    Rotation2d.fromRadians(inputs[1].turnAbsolutePositionRad)),
            new SwerveModuleState(
                    inputs[2].drivePositionRad * WHEEL_RADIUS_METERS,
                    Rotation2d.fromRadians(inputs[2].turnAbsolutePositionRad)),
            new SwerveModuleState(
                    inputs[3].drivePositionRad * WHEEL_RADIUS_METERS,
                    Rotation2d.fromRadians(inputs[3].turnAbsolutePositionRad)));
  }

  /** Run open loop based on stick positions. */
  public void driveArcade(double xSpeed, double zRotation) {
    var speeds = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed * 12, zRotation * 12, 0.0));

    for (int i = 0; i < speeds.length; i++) {
      var speed = speeds[i];
      moduleIOs[i].setDriveVoltage(speed.speedMetersPerSecond);
      var rad = speed.angle.getRadians();

      var lastVel = anglePid.getSetpoint().velocity;
      var t = Timer.getFPGATimestamp();
      var dt = t - lastTime;
      var pid = anglePid.calculate(inputs[i].turnPositionRad, rad);
      var anglePower = -(pid) + angleFF.calculate(
              lastVel,
              anglePid.getSetpoint().velocity,
              0.02//minOf(dt,0.001) // stop possible divide by zero?
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
