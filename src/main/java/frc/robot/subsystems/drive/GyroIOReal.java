package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2_Faults;

public class GyroIOReal implements GyroIO {
    Pigeon2 pigeon2;

    public GyroIOReal(int deviceNumber, String canbus) {
        this.pigeon2 = new Pigeon2(deviceNumber, canbus);
        this.configFactoryDefault();
    }

    public GyroIOReal(int deviceNumber) {
        this.pigeon2 = new Pigeon2(deviceNumber);
        this.configFactoryDefault();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        GyroIO.super.updateInputs(inputs);
        Pigeon2_Faults faults = new Pigeon2_Faults();
        pigeon2.getFaults(faults);
        pigeon2.getBiasedMagnetometer(inputs.biasedMagnetometer);
        pigeon2.getRawMagnetometer(inputs.rawMagnetometer);
        inputs.compassHeading = pigeon2.getCompassHeading();
        inputs.temp = pigeon2.getTemp();
        inputs.uptime = pigeon2.getUpTime();
        pigeon2.getGravityVector(inputs.gravityVector);
        inputs.pitch = pigeon2.getPitch();
        inputs.roll = pigeon2.getRoll();
        inputs.yaw = pigeon2.getYaw();
    }

    @Override
    public void configFactoryDefault() {
        pigeon2.configFactoryDefault();
    }

    @Override
    public void zeroGyroBias(int ms) {
        pigeon2.zeroGyroBiasNow(ms);
    }
}
