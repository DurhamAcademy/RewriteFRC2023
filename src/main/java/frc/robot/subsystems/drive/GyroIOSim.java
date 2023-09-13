package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.subsystems.drive.Drive.WHEEL_RADIUS_METERS;

public class GyroIOSim implements GyroIO {
    ModuleIO.ModuleIOInputs[] inputs;
    Translation2d[] modulePositions;
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    double yaw = 0.0;

    public GyroIOSim(Translation2d[] modulePositions) {
        this.inputs = new ModuleIO.ModuleIOInputs[4];
        this.modulePositions = modulePositions;
        this.kinematics = new SwerveDriveKinematics(modulePositions);
        this.odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(0.0), new SwerveModulePosition[]{
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d())});
    }

    public void setInput(int a, ModuleIO.ModuleIOInputs input) {
        inputs[a] = input;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        var i = new SwerveModuleState[]{
                new SwerveModuleState(
                        this.inputs[0].driveVelocityRadPerSec * WHEEL_RADIUS_METERS,
                        Rotation2d.fromRadians(this.inputs[0].turnAbsolutePositionRad)),
                new SwerveModuleState(
                        this.inputs[1].driveVelocityRadPerSec * WHEEL_RADIUS_METERS,
                        Rotation2d.fromRadians(this.inputs[1].turnAbsolutePositionRad)),
                new SwerveModuleState(
                        this.inputs[2].driveVelocityRadPerSec * WHEEL_RADIUS_METERS,
                        Rotation2d.fromRadians(this.inputs[2].turnAbsolutePositionRad)),
                new SwerveModuleState(
                        this.inputs[3].driveVelocityRadPerSec * WHEEL_RADIUS_METERS,
                        Rotation2d.fromRadians(this.inputs[3].turnAbsolutePositionRad))
        };

        yaw += Rotation2d.fromRadians(kinematics.toChassisSpeeds(i).omegaRadiansPerSecond * .02).getDegrees();
        inputs.yaw = yaw;
    }
}
