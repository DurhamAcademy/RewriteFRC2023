package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(GyroIOInputs inputs) {

    }

    default void configFactoryDefault() {
    }

    default void zeroGyroBias(int ms) {
    }

    default void zeroGyroBias() {
        this.zeroGyroBias(0);
    }

    @AutoLog
    class GyroIOInputs {
        double pitch = 0.0;
        double roll = 0.0;
        double yaw = 0.0;
        double[] gravityVector = new double[0];
        double uptime = 0.0;
        double temp = 0.0;
        double compassHeading = 0.0;
        short[] rawMagnetometer = new short[0];
        short[] biasedMagnetometer = new short[0];
    }
}

