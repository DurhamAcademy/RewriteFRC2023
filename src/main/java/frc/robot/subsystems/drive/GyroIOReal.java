package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2_Faults;

public class GyroIOReal implements GyroIO {
    Pigeon2 pigeon2;

    public GyroIOReal(int deviceNumber, String canbus) {
        this.pigeon2 = new Pigeon2(deviceNumber, canbus);
        this.configFactoryDefault();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        GyroIO.super.updateInputs(inputs);
        Pigeon2_Faults faults = new Pigeon2_Faults();
        pigeon2.getFaults(faults);
        inputs.FaultAccelFault = faults.AccelFault;
        inputs.FaultGyroFault = faults.GyroFault;
        inputs.FaultHardwareFault = faults.HardwareFault;
        inputs.FaultMagnetometerFault = faults.MagnetometerFault;
        inputs.FaultAPIError = faults.APIError;
        inputs.FaultBootIntoMotion = faults.BootIntoMotion;
        inputs.FaultResetDuringEn = faults.ResetDuringEn;
        inputs.FaultSaturatedAccel = faults.SaturatedAccel;
        inputs.FaultSaturatedMag = faults.SaturatedMag;
        inputs.FaultSaturatedRotVelocity = faults.SaturatedRotVelocity;
        inputs.FaultUnderVoltage = faults.UnderVoltage;

        Pigeon2_Faults stickyFaults = new Pigeon2_Faults();
        pigeon2.getFaults(stickyFaults);
        inputs.StickyFaultAccelFault = stickyFaults.AccelFault;
        inputs.StickyFaultGyroFault = stickyFaults.GyroFault;
        inputs.StickyFaultHardwareFault = stickyFaults.HardwareFault;
        inputs.StickyFaultMagnetometerFault = stickyFaults.MagnetometerFault;
        inputs.StickyFaultAPIError = stickyFaults.APIError;
        inputs.StickyFaultBootIntoMotion = stickyFaults.BootIntoMotion;
        inputs.StickyFaultResetDuringEn = stickyFaults.ResetDuringEn;
        inputs.StickyFaultSaturatedAccel = stickyFaults.SaturatedAccel;
        inputs.StickyFaultSaturatedMag = stickyFaults.SaturatedMag;
        inputs.StickyFaultSaturatedRotVelocity = stickyFaults.SaturatedRotVelocity;
        inputs.StickyFaultUnderVoltage = stickyFaults.UnderVoltage;

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
}
