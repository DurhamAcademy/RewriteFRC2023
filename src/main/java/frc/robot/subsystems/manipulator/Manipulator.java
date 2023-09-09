package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.manipulator.SetManipulatorSpeed;

public class Manipulator extends SubsystemBase {
    public ManipulatorIO io;
    public ManipulatorIO.ManipulatorIOInputs inputs;
    public int motorId;

    public Manipulator(ManipulatorIO io, int motorId){
        this.io = io;
        this.motorId = motorId;

        //TODO put this in constructor or init @everett
        CANSparkMax motor = new CANSparkMax(motorId, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    }

    public void init() {
        setDefaultCommand(new SetManipulatorSpeed(this, 0.05));
        //TODO probably am not supposed to use init as it does nothing lol
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

}
