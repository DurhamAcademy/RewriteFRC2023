package frc.robot.subsystems.battery;

import edu.wpi.first.wpilibj.simulation.BatterySim;

import java.util.ArrayList;
import java.util.List;

public class BatteryIOSim implements BatteryIO {

    List<Double> currents = new ArrayList<>();

    @Override
    public void updateInputs(BatteryIOInputs inputs) {
        var total = new double[currents.size() == 0 ? 1 : currents.size()];
        if (currents.size() == 0) total[0] = 0.0;
        for (int i = 0; i < currents.size(); i++) {
            total[i] = currents.get(i);
        }
        inputs.voltage = BatterySim.calculateDefaultBatteryLoadedVoltage(total);
        currents.clear();
    }

    public void addCurrent(double current) {
        this.currents.add(current);
    }
}
