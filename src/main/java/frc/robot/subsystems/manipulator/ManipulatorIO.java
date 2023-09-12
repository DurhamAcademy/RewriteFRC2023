package frc.robot.subsystems.manipulator;

import frc.robot.subsystems.flywheel.FlywheelIO;
import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {
    @AutoLog
    public static class ManipulatorIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {0.0};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ManipulatorIO.ManipulatorIOInputs inputs) {
    }

    /** Run closed loop at the specified velocity. */
    public default void setVelocity(double velocityRadPerSec, double ffVolts) {
    }

    /** Stop in open loop. */
    public default void stop() {
    }
    public default void setPercentage(double percentage) {
    }
    
    public default void setVoltage(double voltage) {
    }
    
    /** Set velocity PID constants. */
}
