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

    @AutoLog
    class GyroIOInputs {
        double pitch;
        double roll;
        double yaw;
        double[] gravityVector;
        double uptime;
        double temp;
        double compassHeading;
        short[] rawMagnetometer;
        short[] biasedMagnetometer;

        boolean FaultAccelFault;
        boolean FaultGyroFault;
        boolean FaultHardwareFault;
        boolean FaultMagnetometerFault;
        boolean FaultAPIError;
        boolean FaultBootIntoMotion;
        boolean FaultResetDuringEn;
        boolean FaultSaturatedAccel;
        boolean FaultSaturatedMag;
        boolean FaultSaturatedRotVelocity;
        boolean FaultUnderVoltage;

        boolean StickyFaultAccelFault;
        boolean StickyFaultGyroFault;
        boolean StickyFaultHardwareFault;
        boolean StickyFaultMagnetometerFault;
        boolean StickyFaultAPIError;
        boolean StickyFaultBootIntoMotion;
        boolean StickyFaultResetDuringEn;
        boolean StickyFaultSaturatedAccel;
        boolean StickyFaultSaturatedMag;
        boolean StickyFaultSaturatedRotVelocity;
        boolean StickyFaultUnderVoltage;

    }
}

