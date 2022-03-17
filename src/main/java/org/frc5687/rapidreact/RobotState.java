package org.frc5687.rapidreact;

import org.frc5687.rapidreact.util.SystemState;
import org.frc5687.rapidreact.util.VisionState;

public class RobotState extends SystemState {
    private VisionState _visionState;

    public RobotState(VisionState visionState) {
        _visionState = visionState;
    }

    public VisionState getVisionState() {
        return _visionState;
    }
}
