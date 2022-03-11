package org.frc5687.rapidreact.util;

import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

public class Limelight {

    private PhotonCamera _camera;
    private boolean _inDriverMode;

    public Limelight(String cameraName){
        _camera = new PhotonCamera(cameraName);
    }

    /**
     * Get the latency of the camera
     * @return latency in ms
     */
    public double getLatency() {
        return _camera.getLatestResult().getLatencyMillis();
    }

    /**
     * Gets the yaw of the target
     * @return yaw in rads
     */
    public double getYaw() {
        if(hasTarget()){
            return Units.degreesToRadians(_camera.getLatestResult().getBestTarget().getYaw());
        }
        return 0.0;
    }

    /**
     * Get the area of the targets
     * @return area
     */
    public double getArea(){
        if (hasTarget()) {
            return _camera.getLatestResult().getBestTarget().getArea();
        }
        return 0.0;
    }

    /**
     * Get the pitch of the target
     * @return pitch
     */
    public double getPitch(){
        if (hasTarget()) {
            return _camera.getLatestResult().getBestTarget().getPitch();
        }
        return 0.0;
    }

    /**
     * Gets the skew of the target
     * @return skew
     */
    public double getSkew(){
        if (hasTarget()) {
            return _camera.getLatestResult().getBestTarget().getSkew();
        }
        return 0.0;
    }

    /**
     * Check to see if the limelight has a target
     * @return has target
     */
    public boolean hasTarget() {
        return _camera.getLatestResult().hasTargets();
    }

    /**
     * Sets the LEDS to on
     */
    public void LEDOn(){
        _camera.setLED(VisionLEDMode.kOn);
    }

    /**
     * Sets the LEDS to off
     */
    public void LEDOff(){
        _camera.setLED(VisionLEDMode.kOff);
    }

    /**
     * Blinks the LEDS
     */
    public void LEDBlink(){
        _camera.setLED(VisionLEDMode.kBlink);
    }

    /**
     * Sets the robot into driver mode
     * @param mode set to driver mode or not
     */
    public void setDriverMode(boolean mode){
        _camera.setDriverMode(mode);
        _inDriverMode = mode;
    }

    /**
     * Returns true if in driver mode
     * @return
     */
    public boolean getDriverMode(){
        return _inDriverMode;
    }
}
//Kilroy Was Here