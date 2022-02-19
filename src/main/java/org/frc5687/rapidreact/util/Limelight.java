package org.frc5687.rapidreact.util;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

public class Limelight {

    private PhotonCamera _camera;
    private boolean _inDriverMode;
    private double yaw = 0.0;
    private double pitch = 0.0;
    private double area = 0.0;

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
            return _camera.getLatestResult().getBestTarget().getYaw();
        }
        return 0.0;
    }

    public double getArea(){
        //Get area of bounding box of target
        return _camera.getLatestResult().getBestTarget().getArea();
    }

    public double getPitch(){
        //Gets realitive pitch of target
        return _camera.getLatestResult().getBestTarget().getPitch();
    }

    public double getSkew(){
        return _camera.getLatestResult().getBestTarget().getSkew();
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