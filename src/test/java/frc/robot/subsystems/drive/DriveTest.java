package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import org.junit.jupiter.api.*;

import java.util.Random;

import static frc.robot.subsystems.drive.Drive.WHEEL_RADIUS_METERS;

class DriveTest {

    /**
     * Should be constant throughout all tests
     */
    Random rand = new Random();

    int updateInputsCallCount = 0;

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
                updateInputsCallCount++;
                try {
                    driveIOInputs = (DriveIOInputs) inputs.clone();
                } catch (CloneNotSupportedException e) {
                    Assertions.fail("Threw CloneNotSupportedException", e);
                } catch (ClassCastException e) {
                    Assertions.fail("Cannot cast clone of inputs to DriveIOInputs", e);
                }
                inputs.gyroYawRad = 1.0;
                inputs.leftPositionRad = rand.nextDouble() * 6;
                inputs.leftVelocityRadPerSec = rand.nextDouble() * 0.5;
                inputs.rightPositionRad = rand.nextDouble() * 6;
                inputs.rightVelocityRadPerSec = rand.nextDouble() * 0.5;
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
    @DisplayName("updateInputs() called once during periodic")
    void callCountTest() {
        drive.periodic();

        Assertions.assertEquals(1, updateInputsCallCount, "update inputs not called only once in periodic (and only periodic)");
    }

    @RepeatedTest(10)
    @DisplayName("drivePercent() Correct voltage test")
    void drivePercent() {
        var newDriveIOLeftPercent = (rand.nextDouble() * 2) - 1;
        var newDriveIORightPercent = (rand.nextDouble() * 2) - 1;

        drive.drivePercent(newDriveIOLeftPercent, newDriveIORightPercent);

        Assertions.assertEquals(newDriveIOLeftPercent * 12.0, driveIOLeftVoltage);
        Assertions.assertEquals(newDriveIORightPercent * 12.0, driveIORightVoltage);
    }

    @RepeatedTest(10)
    @DisplayName("driveArcade() Correct voltage test")
    void driveArcade() {
        driveIOLeftVoltage = 0.0;
        driveIORightVoltage = 0.0;
        var newDriveIOLeftVoltage = (rand.nextDouble() * 2) - 1;
        var newDriveIORightVoltage = (rand.nextDouble() * 2) - 1;

        drive.driveArcade(newDriveIOLeftVoltage, newDriveIORightVoltage);

        var speeds = DifferentialDrive.arcadeDriveIK(newDriveIOLeftVoltage, newDriveIORightVoltage, true);

        Assertions.assertEquals(speeds.left * 12.0, driveIOLeftVoltage);
        Assertions.assertEquals(speeds.right * 12.0, driveIORightVoltage);
    }

    @Test
    @DisplayName("stop() Correct voltage test")
    void stop() {
        driveIOLeftVoltage = 3.0;
        driveIORightVoltage = 5.2;

        drive.stop();

        Assertions.assertEquals(0.0, driveIOLeftVoltage);
        Assertions.assertEquals(0.0, driveIORightVoltage);
    }

    @Test
    @DisplayName("getPose() Getter Test")
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
    @DisplayName("getLeftPositionMeters() Getter Test")
    void getLeftPositionMeters() {
        Assertions.assertEquals(0.0, drive.getLeftPositionMeters());

        drive.periodic();

        Assertions.assertEquals(newDriveIOInputs.leftPositionRad * WHEEL_RADIUS_METERS,
                drive.getLeftPositionMeters());
    }

    @Test
    @DisplayName("getRightPositionMeters() Getter Test")
    void getRightPositionMeters() {
        Assertions.assertEquals(0.0, drive.getRightPositionMeters());

        drive.periodic();

        Assertions.assertEquals(newDriveIOInputs.rightPositionRad * WHEEL_RADIUS_METERS,
                drive.getRightPositionMeters());
    }

    @Test
    @DisplayName("getLeftVelocityMeters() Getter Test")
    void getLeftVelocityMeters() {
        Assertions.assertEquals(0.0, drive.getLeftVelocityMeters());

        drive.periodic();

        Assertions.assertEquals(newDriveIOInputs.leftVelocityRadPerSec * WHEEL_RADIUS_METERS,
                drive.getLeftVelocityMeters());
    }

    @Test
    @DisplayName("getRightVelocityMeters() Getter Test")
    void getRightVelocityMeters() {
        Assertions.assertEquals(0.0, drive.getRightVelocityMeters());

        drive.periodic();

        Assertions.assertEquals(newDriveIOInputs.rightVelocityRadPerSec * WHEEL_RADIUS_METERS,
                drive.getRightVelocityMeters());
    }
}