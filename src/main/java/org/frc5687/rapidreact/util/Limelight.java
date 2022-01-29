package org.frc5687.rapidreact.util;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;


public class Limelight {

    private final PhotonCamera _camera;
    public boolean _inDriverMode;
    public double yaw = 0.0;
    public double pitch = 0.0;
    public double area = 0.0;

    public Limelight(String cameraName){
        _camera = new PhotonCamera(cameraName);
    }

    public void update(){
        PhotonPipelineResult result = _camera.getLatestResult();
    }
}
