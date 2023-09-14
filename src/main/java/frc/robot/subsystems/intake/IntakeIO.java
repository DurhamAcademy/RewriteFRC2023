package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    default void updateInputs(IntakeIOInputs inputs) {
    }

    default void setModeEncoderPosition(double position) {
    }

    default void setModeVoltage(double voltage) {
    }

    default void setIntakePercentage(double percentage) {
    }

    default void setIntakeVoltage(double voltage) {
    }

    default void setDeployVoltage(double voltage) {
    }

    @AutoLog
    class IntakeIOInputs {
        double modeMotorOutputCurrent;
        double modeEncoderPosition;

        double intakeMotorOutputCurrent;
        double intakeMotorSpeed;

        double deployMotorOutputCurrent;
        double deployEncoderPosition;
    }
}
