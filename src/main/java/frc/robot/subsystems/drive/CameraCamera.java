package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

public class CameraCamera extends PhotonCamera {
    private VisionIO.VisionIOInputs inputs;

    public CameraCamera() {
        super("Fake");
    }

    public void setInputs(VisionIO.VisionIOInputs inputs) {
        this.inputs = inputs;
    }

    @Override
    public Optional<Matrix<N3, N3>> getCameraMatrix() {
        return inputs.cameraMatrix;
    }

    @Override
    public PhotonPipelineResult getLatestResult() {
        return inputs.cameraResult;
    }

    @Override
    public Optional<Matrix<N5, N1>> getDistCoeffs() {
        return inputs.distanceCoefficients;
    }
}
