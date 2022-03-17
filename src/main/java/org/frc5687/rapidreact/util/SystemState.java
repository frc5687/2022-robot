package org.frc5687.rapidreact.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SystemState {
    protected long _millis;

    public SystemState() {
        _millis = System.currentTimeMillis();
    }

    public long getMillis() {
        return _millis;
    }

    public void updateDashboard(String prefix) {
        SmartDashboard.putNumber(prefix + "/millis", _millis);
    }
}
