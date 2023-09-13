package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.Timer;
import org.junit.jupiter.api.*;

import java.io.IOException;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

class DriveTest {

    /**
     * Should be constant throughout all tests
     */
    Random rand = new Random();

    int updateInputsCallCount = 0;

    ModuleIO.ModuleIOInputs driveIOInputs;
    ModuleIO.ModuleIOInputs newDriveIOInputs;

    double driveIOTurnVoltage;
    double driveIODriveVoltage;

    Drive drive;

    @BeforeEach
    void setUp() throws IOException {
        driveIOInputs = null;
        newDriveIOInputs = null;
        var moduleIOs = new ModuleIO[4];
        for (int i = 0; i < 4; i++) {
            moduleIOs[i] = new ModuleIO() {
                @Override
                public void updateInputs(ModuleIOInputs inputs) {
                    updateInputsCallCount++;
                    try {
                        driveIOInputs = (ModuleIOInputs) inputs.clone();
                    } catch (CloneNotSupportedException e) {
                        Assertions.fail("Threw CloneNotSupportedException", e);
                    } catch (ClassCastException e) {
                        Assertions.fail("Cannot cast clone of inputs to DriveIOInputs", e);
                    }
                    inputs.turnAbsolutePositionRad = rand.nextDouble() * 6;
                    inputs.turnPositionRad = inputs.turnAbsolutePositionRad + rand.nextDouble() * 0.2;
                    inputs.turnAbsolutePositionRad = rand.nextDouble() * 0.5;
                    inputs.drivePositionRad = rand.nextDouble() * 6;
                    inputs.driveVelocityRadPerSec = rand.nextDouble() * 0.5;
                    try {
                        newDriveIOInputs = (ModuleIOInputs) inputs.clone();
                    } catch (CloneNotSupportedException e) {
                        Assertions.fail("Threw CloneNotSupportedException", e);
                    } catch (ClassCastException e) {
                        Assertions.fail("Cannot cast clone of inputs to DriveIOInputs", e);
                    }
                }

                @Override
                public void setTurnVoltage(double volts) {
                    driveIOTurnVoltage = volts;
                }

                @Override
                public void setDriveVoltage(double volts) {
                    driveIODriveVoltage = volts;
                }
            };
        }
        drive = new Drive(moduleIOs[0], moduleIOs[1], moduleIOs[2], moduleIOs[3], new GyroIO() {
        }, new VisionIO() {
        });
        driveIODriveVoltage = 0.0;
        driveIOTurnVoltage = 0.0;
    }

    @Test
    @DisplayName("updateInputs() called once during periodic")
    void callCountTest() {
        Logger.getGlobal().log(Level.INFO, "updateInputsCallCount", updateInputsCallCount);

        drive.periodic();

        Assertions.assertEquals(4, updateInputsCallCount, "update inputs not called only once per module in periodic (and only periodic)");
    }

    @Test
    @DisplayName("stop() Correct voltage test")
    void stop() {
        driveIOTurnVoltage = 3.0;
        driveIODriveVoltage = 5.2;

        drive.stop();

        Assertions.assertEquals(0.0, driveIOTurnVoltage);
        Assertions.assertEquals(0.0, driveIODriveVoltage);
    }


    @Test
    @DisplayName("getPose() X Getter Test")
    void getPoseX() {
        Assertions.assertEquals(0.0, drive.getPose().getX());

        drive.testMode = true;
        drive.testTime = Timer.getFPGATimestamp();
        for (int i = 0; i < 50; i++) {
            drive.periodic();
            drive.testTime += 0.02;
        }

        System.out.println(drive.getPose());
        Assertions.assertNotEquals(0.0, drive.getPose().getX());
    }

    @Test
    @DisplayName("getPose() Y Getter Test")
    void getPoseY() {
        Assertions.assertEquals(0.0, drive.getPose().getY());

        drive.testMode = true;
        drive.testTime = Timer.getFPGATimestamp();
        for (int i = 0; i < 50; i++) {
            drive.periodic();
            drive.testTime += 0.02;
        }

        System.out.println(drive.getPose());
        Assertions.assertNotEquals(0.0, drive.getPose().getY());
    }

    @Test
    @Disabled("Rotation does not work")
    @DisplayName("getPose() Rotation Getter Test")
    void getPoseRot() {
        Assertions.assertEquals(0.0, drive.getPose().getRotation().getRadians());

        drive.testMode = true;
        drive.testTime = Timer.getFPGATimestamp();
        for (int i = 0; i < 50; i++) {
            drive.periodic();
            drive.testTime += 0.02;
        }

        System.out.println(drive.getPose());
        Assertions.assertNotEquals(0.0, drive.getPose().getRotation().getRadians());
    }
}