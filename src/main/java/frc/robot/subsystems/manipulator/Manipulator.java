package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.Debouncer;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
    public ManipulatorIO io;
    public ManipulatorIO.ManipulatorIOInputs inputs;

    public Manipulator(int motorId){
        CANSparkMax motor = new CANSparkMax(motorId, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        /*Debouncer motorIdleDebouncer = new Debouncer(0.075);
        double lastPercent = 0.0;
        CANSparkMax.IdleMode lastIdleMode = CANSparkMax.IdleMode.kBrake;*/
        //TODO figure out if this is important
    }

    public void init() {
        //TODO add default command for manipulator
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
        //TODO make this work idk io stuff
    }

}
