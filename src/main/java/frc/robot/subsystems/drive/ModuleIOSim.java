package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.subsystems.battery.BatteryIO;
import frc.robot.subsystems.battery.BatteryIOInputsAutoLogged;
import frc.robot.subsystems.battery.BatteryIOSim;

import java.util.Optional;

public class ModuleIOSim implements ModuleIO {
    private Optional<BatteryIO.BatteryIOInputs> batteryInputs = Optional.empty();
    private Optional<Integer> id = Optional.empty();
    private Optional<GyroIOSim> gyroSim = Optional.empty();
    private Optional<BatteryIOSim> batterySim = Optional.empty();

    private final FlywheelSim driveSim =
            new FlywheelSim(DCMotor.getFalcon500(1), 6.75, 0.025);
    private final FlywheelSim turnSim =
            new FlywheelSim(DCMotor.getFalcon500(1), 150.0 / 7.0, 0.004096955);

    private double turnRelativePositionRad = 0.0;
    private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim(int id, GyroIOSim gyroSim) {
        this.id = Optional.of(id);
        this.gyroSim = Optional.ofNullable(gyroSim);

    }

    public ModuleIOSim(int id, GyroIOSim gyroSim, BatteryIOSim batteryIO, BatteryIOInputsAutoLogged batteryInputs) {
        this.id = Optional.of(id);
        this.gyroSim = Optional.ofNullable(gyroSim);
        this.batterySim = Optional.ofNullable(batteryIO);
        this.batteryInputs = Optional.ofNullable(batteryInputs);
    }

    public ModuleIOSim() {
    }

    public void updateInputs(ModuleIOInputs inputs) {
        driveSim.update(Constants.loopPeriodSecs);
        turnSim.update(Constants.loopPeriodSecs);
        batterySim.ifPresent(batteryIO -> batteryIO.addCurrent(driveSim.getCurrentDrawAmps()));
        batterySim.ifPresent(batteryIO -> batteryIO.addCurrent(turnSim.getCurrentDrawAmps()));

        double angleDiffRad =
                turnSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs;
        turnRelativePositionRad += angleDiffRad;
        turnAbsolutePositionRad += angleDiffRad;
        while (turnAbsolutePositionRad < 0) {
            turnAbsolutePositionRad += 2.0 * Math.PI;
        }
        while (turnAbsolutePositionRad > 2.0 * Math.PI) {
            turnAbsolutePositionRad -= 2.0 * Math.PI;
        }

        inputs.drivePositionRad = inputs.drivePositionRad
                + (driveSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveOutputVoltage = driveAppliedVolts;
        inputs.driveCurrentAmps =
                new double[]{Math.abs(driveSim.getCurrentDrawAmps())};
        inputs.driveTempCelsius = new double[]{};

        inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
        inputs.turnPositionRad = turnRelativePositionRad;
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnOutputVoltage = turnAppliedVolts;
        inputs.turnCurrentAmps =
                new double[]{Math.abs(turnSim.getCurrentDrawAmps())};
        inputs.turnTempCelsius = new double[]{};
        gyroSim.ifPresent(gyroIOSim -> gyroIOSim.setInput(id.orElseThrow(), inputs));
    }

    public void setDriveVoltage(double volts) {
        double voltage = (batteryInputs.isPresent()) ? batteryInputs.get().voltage : 12.0;
        driveAppliedVolts = MathUtil.clamp(volts, -voltage, voltage);
        driveSim.setInputVoltage(driveAppliedVolts - (driveSim.getAngularVelocityRadPerSec() * 0.02));
    }

    public void setTurnVoltage(double volts) {
        double voltage = (batteryInputs.isPresent()) ? batteryInputs.get().voltage : 12.0;
        turnAppliedVolts = -MathUtil.clamp(volts, -voltage, voltage);
        turnSim.setInputVoltage(turnAppliedVolts - (turnSim.getAngularVelocityRadPerSec() * 0.005));
    }
}
