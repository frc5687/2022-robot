package org.frc5687.rapidreact.util;

public class VisionState extends SystemState {

    private double _angle;
    private double _distance;

    public VisionState(double angle, double distance) {
        super();
        _angle = angle;
        _distance = distance;
    }
    public double getAngle() {
       return _angle;
    }
    public double getDistance() {
        return _distance;
    }
}
