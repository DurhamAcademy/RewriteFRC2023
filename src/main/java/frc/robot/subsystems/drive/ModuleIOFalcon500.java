package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.util.Units;

import static com.ctre.phoenix.motorcontrol.NeutralMode.Brake;
import static com.ctre.phoenix.motorcontrol.NeutralMode.Coast;

public class ModuleIOFalcon500 implements ModuleIO {
  private static final double DRIVE_GEAR_RATIO = 6.75;
  private static final double TURN_GEAR_RATIO = 6.0;
  private static final double TICKS_PER_REV = 2048;

  private final WPI_TalonFX turnMotor;
  private final WPI_TalonFX driveMotor;

  private final CANCoder turnEncoder;

  public ModuleIOFalcon500(int turnCanId,
                           int driveCanId,
                           boolean turnInvert,
                           boolean driveInvert,
                           int turnEncoderCanId) {
    turnMotor = new WPI_TalonFX(turnCanId);
    driveMotor = new WPI_TalonFX(driveCanId);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.voltageCompSaturation = 12.0;
    config.statorCurrLimit.enable = true;
    config.statorCurrLimit.currentLimit = 40;

    turnMotor.configAllSettings(config);
    driveMotor.configAllSettings(config);

    driveMotor.setNeutralMode(Brake);
    turnMotor.setNeutralMode(Brake);

    turnMotor.setInverted(turnInvert);
    driveMotor.setInverted(driveInvert);

    turnEncoder = new CANCoder(turnEncoderCanId);
    turnEncoder.configFactoryDefault();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = Units.rotationsToRadians(
            driveMotor.getSelectedSensorPosition() / TICKS_PER_REV / DRIVE_GEAR_RATIO);
    inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
            driveMotor.getSelectedSensorVelocity() * 10 / TICKS_PER_REV / DRIVE_GEAR_RATIO);
    inputs.driveOutputVoltage = driveMotor.getMotorOutputVoltage();
    inputs.driveCurrentAmps =
            new double[]{driveMotor.getStatorCurrent()};
    inputs.driveTempCelsius =
            new double[]{driveMotor.getTemperature()};

    inputs.turnPositionRad = Units.rotationsToRadians(
            turnMotor.getSelectedSensorPosition() / TICKS_PER_REV / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
            turnMotor.getSelectedSensorVelocity() * 10 / TICKS_PER_REV / TURN_GEAR_RATIO);
    inputs.turnCurrentAmps =
            new double[]{turnMotor.getStatorCurrent()};
    inputs.turnTempCelsius =
            new double[]{turnMotor.getTemperature()};

    inputs.turnAbsolutePositionRad = Units.degreesToRadians(turnEncoder.getAbsolutePosition());
  }

  /**
   * Instruct the drive motor to run at the specified voltage.
   *
   * @param volts Instructed voltage
   */
  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setVoltage(volts);
  }

  /**
   * Instruct the drive motor to run at the specified voltage.
   *
   * @param volts Instructed voltage
   */
  @Override
  public void setTurnVoltage(double volts) {
    turnMotor.setVoltage(volts);

  }

  /**
   * Enable or disable brake mode on the drive motor.
   */
  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveMotor.setNeutralMode((enable) ? Brake : Coast);
  }

  /**
   * Enable or disable brake mode on the turn motor.
   */
  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnMotor.setNeutralMode((enable) ? Brake : Coast);
  }
}
