package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(ModuleIOInputs inputs) {
    }

    /**
     * Run the drive motor at the specified voltage.
     */
    default void setDriveVoltage(double volts) {
    }

    /**
     * Run the turn motor at the specified voltage.
     */
    default void setTurnVoltage(double volts) {
    }

    /**
     * Enable or disable brake mode on the drive motor.
     */
    default void setDriveBrakeMode(boolean enable) {
    }

    /**
     * Enable or disable brake mode on the turn motor.
     */
    default void setTurnBrakeMode(boolean enable) {
    }

    @AutoLog
    class ModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveOutputVoltage = 0.0;
        public double[] driveCurrentAmps = new double[]{};
        public double[] driveTempCelsius = new double[]{};

        public double turnAbsolutePositionRad = 0.0;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnOutputVoltage = 0.0;
        public double[] turnCurrentAmps = new double[]{};
        public double[] turnTempCelsius = new double[]{};

        @Override
        protected Object clone() throws CloneNotSupportedException {
            Object clone = super.clone();
            ModuleIOInputsAutoLogged copy = new ModuleIOInputsAutoLogged();
            copy.drivePositionRad = this.drivePositionRad;
            copy.driveVelocityRadPerSec = this.driveVelocityRadPerSec;
            copy.driveOutputVoltage = this.driveOutputVoltage;
            copy.driveCurrentAmps = this.driveCurrentAmps.clone();
            copy.driveTempCelsius = this.driveTempCelsius.clone();
            copy.turnAbsolutePositionRad = this.turnAbsolutePositionRad;
            copy.turnPositionRad = this.turnPositionRad;
            copy.turnVelocityRadPerSec = this.turnVelocityRadPerSec;
            copy.turnOutputVoltage = this.turnOutputVoltage;
            copy.turnCurrentAmps = this.turnCurrentAmps.clone();
            copy.turnTempCelsius = this.turnTempCelsius.clone();
            return copy;
        }
    }
}

