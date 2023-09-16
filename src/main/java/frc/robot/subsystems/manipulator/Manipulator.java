package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.manipulator.SetManipulatorSpeed;

public class Manipulator extends SubsystemBase {
    public ManipulatorIO io;
    public ManipulatorIO.ManipulatorIOInputs inputs;

    public Manipulator(ManipulatorIO io){
        this.io = io;
        
       setDefaultCommand(new SetManipulatorSpeed(this, 0.05));
    }



    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setMotorVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void setMotorPercent(double percentage) {
        io.setPercentage(percentage);
    }
}
