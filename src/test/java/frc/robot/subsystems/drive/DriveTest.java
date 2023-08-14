package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static frc.robot.subsystems.drive.Drive.WHEEL_RADIUS_METERS;

class DriveTest {
    DriveIO.DriveIOInputs driveIOInputs;
    DriveIO.DriveIOInputs newDriveIOInputs;

    double driveIOLeftVoltage;
    double driveIORightVoltage;

    DriveIO driveIO;

    Drive drive;

    @BeforeEach
    void setUp() {
        driveIOInputs = null;
        newDriveIOInputs = null;
        driveIO = new DriveIO() {
            @Override
            public void updateInputs(DriveIOInputs inputs) {
                try {
                    driveIOInputs = (DriveIOInputs) inputs.clone();
                } catch (CloneNotSupportedException e) {
                    Assertions.fail("Threw CloneNotSupportedException", e);
                } catch (ClassCastException e) {
                    Assertions.fail("Cannot cast clone of inputs to DriveIOInputs", e);
                }
                inputs.gyroYawRad = 1.0;
                inputs.leftPositionRad = 3.6;
                inputs.leftVelocityRadPerSec = 0.4;
                inputs.rightPositionRad = 4.11;
                inputs.rightVelocityRadPerSec = 0.1;
                try {
                    newDriveIOInputs = (DriveIOInputs) inputs.clone();
                } catch (CloneNotSupportedException e) {
                    Assertions.fail("Threw CloneNotSupportedException", e);
                } catch (ClassCastException e) {
                    Assertions.fail("Cannot cast clone of inputs to DriveIOInputs", e);
                }
            }

            @Override
            public void setVoltage(double leftVolts, double rightVolts) {
                driveIOLeftVoltage = leftVolts;
                driveIORightVoltage = rightVolts;
            }
        };
        drive = new Drive(driveIO);
        driveIORightVoltage = 0.0;
        driveIOLeftVoltage = 0.0;
    }

    @Test
    void drivePercent() {
        var rand = new Random();
        var newDriveIOLeftPercent = rand.nextDouble();
        var newDriveIORightPercent = rand.nextDouble();

        drive.drivePercent(newDriveIOLeftPercent, newDriveIORightPercent);

        Assertions.assertEquals(newDriveIOLeftPercent * 12.0, driveIOLeftVoltage);
        Assertions.assertEquals(newDriveIORightPercent * 12.0, driveIORightVoltage);
    }

    @Test
    void driveArcade() {
        driveIOLeftVoltage = 0.0;
        driveIORightVoltage = 0.0;
        drive.driveArcade(0.7, 0.4);

        var speeds = DifferentialDrive.arcadeDriveIK(0.7, 0.4, true);

        Assertions.assertEquals(speeds.left * 12.0, driveIOLeftVoltage);
        Assertions.assertEquals(speeds.right * 12.0, driveIORightVoltage);
    }

    @Test
    void stop() {
        driveIOLeftVoltage = 3.0;
        driveIORightVoltage = 5.2;

        drive.stop();

        Assertions.assertEquals(0.0, driveIOLeftVoltage);
        Assertions.assertEquals(0.0, driveIORightVoltage);
    }

    @Test
    void getPose() {
        Assertions.assertEquals(0.0, drive.getPose().getX());
        Assertions.assertEquals(0.0, drive.getPose().getY());
        Assertions.assertEquals(0.0, drive.getPose().getRotation().getRadians());
        drive.periodic();
        Assertions.assertNotEquals(0.0, drive.getPose().getX());
        Assertions.assertNotEquals(0.0, drive.getPose().getY());
        Assertions.assertNotEquals(0.0, drive.getPose().getRotation().getRadians());
    }

    @Test
    void getLeftPositionMeters() {
        Assertions.assertEquals(0.0, drive.getLeftPositionMeters());

        drive.periodic();

        Assertions.assertEquals(newDriveIOInputs.leftPositionRad * WHEEL_RADIUS_METERS,
                drive.getLeftPositionMeters());
    }

    @Test
    void getRightPositionMeters() {
        Assertions.assertEquals(0.0, drive.getRightPositionMeters());

        drive.periodic();

        Assertions.assertEquals(newDriveIOInputs.rightPositionRad * WHEEL_RADIUS_METERS,
                drive.getRightPositionMeters());
    }

    @Test
    void getLeftVelocityMeters() {
        Assertions.assertEquals(0.0, drive.getLeftVelocityMeters());

        drive.periodic();

        Assertions.assertEquals(newDriveIOInputs.leftVelocityRadPerSec * WHEEL_RADIUS_METERS,
                drive.getLeftVelocityMeters());
    }

    @Test
    void getRightVelocityMeters() {
        Assertions.assertEquals(0.0, drive.getRightVelocityMeters());

        drive.periodic();

        Assertions.assertEquals(newDriveIOInputs.rightVelocityRadPerSec * WHEEL_RADIUS_METERS,
                drive.getRightVelocityMeters());
    }
}