package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  class DriveIOInputs {
    public double leftPositionRad = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double rightPositionRad = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double gyroYawRad = 0.0;

    @Override
    protected Object clone() throws CloneNotSupportedException {
      return super.clone();
    }
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(DriveIOInputs inputs) {
  }

  /** Run open loop at the specified voltage. */
  default void setVoltage(double leftVolts, double rightVolts) {
  }
}
