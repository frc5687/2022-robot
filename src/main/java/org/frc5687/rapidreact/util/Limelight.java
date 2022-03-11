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


    public double getLatency() {
        //Gets the latency of the camera
        return _camera.getLatestResult().getLatencyMillis();
    }

    public double getYaw() {
        //Gets the realitive yaw of the target
        if(hasTarget()){
            return Units.degreesToRadians(_camera.getLatestResult().getBestTarget().getYaw());
        }
        return 0.0;
    }

    public double getArea(){
        //Get area of bounding box of target
        if (hasTarget()) {
            return _camera.getLatestResult().getBestTarget().getArea();
        }
        return 0.0;
    }

    public double getPitch(){
        //Gets realitive pitch of target
        if (hasTarget()) {
            return _camera.getLatestResult().getBestTarget().getPitch();
        }
        return 0.0;
    }

    public double getSkew(){
        if (hasTarget()) {
            return _camera.getLatestResult().getBestTarget().getSkew();
        }
        return 0.0;
    }

    public boolean hasTarget() {
        //Does the limelight have a target
        return _camera.getLatestResult().hasTargets();
    }

    public void LEDOn(){
        _camera.setLED(VisionLEDMode.kOn);
    }

    public void LEDOff(){
        _camera.setLED(VisionLEDMode.kOff);
    }

    public void LEDBlink(){
        _camera.setLED(VisionLEDMode.kBlink);
    }

    public void setDriverMode(boolean mode){
        _camera.setDriverMode(mode);
        _inDriverMode = mode;
    }

    public boolean getMode(){
        return _inDriverMode;
    }
}
//Kilroy Was Here