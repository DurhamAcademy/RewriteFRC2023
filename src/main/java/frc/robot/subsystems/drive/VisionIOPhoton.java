package frc.robot.subsystems.drive;

import org.photonvision.PhotonCamera;

public class VisionIOPhoton implements VisionIO {
    PhotonCamera camera = new PhotonCamera("OV9281");

    public void updateInputs(VisionIOInputs inputs) {
        var latest = camera.getLatestResult();
        inputs.driverMode = camera.getDriverMode();
        inputs.hasTargets = latest.hasTargets();
        if (inputs.hasTargets) {
            inputs.bestTarget = latest.getBestTarget();
        }
        inputs.targets = latest.getTargets();
        inputs.latencyMillis = latest.getLatencyMillis();
        inputs.timestampSeconds = latest.getTimestampSeconds();
        inputs.LEDMode = camera.getLEDMode();
        inputs.cameraMatrix = camera.getCameraMatrix();
        inputs.cameraResult = latest;
        inputs.distanceCoefficients = camera.getDistCoeffs();
    }
}
