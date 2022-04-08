/* Team 5687 (C)2022 */
package org.frc5687.rapidreact.util;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import org.frc5687.rapidreact.Constants;

public final class PeriodicManager {
    private final List<OutlierPeriodic> _periods = new ArrayList<>();
    private double _prevTimestamp;
    private double _dt;

    private final Notifier _periodicThread =
            new Notifier(
                    () -> {
                        synchronized (PeriodicManager.this) {
                            final double timestamp = Timer.getFPGATimestamp();
                            _dt = timestamp - _prevTimestamp;
                            _prevTimestamp = timestamp;
                            _periods.forEach(p -> p.controlPeriodic(timestamp, _dt));
                            _periods.forEach(p -> p.dataPeriodic(timestamp, _dt));
                        }
                    });

    public PeriodicManager(OutlierPeriodic... periods) {
        this._periods.addAll(List.of(periods));
    }

    public void startPeriodic() {
        _periodicThread.startPeriodic(Constants.PERIODIC_PERIOD);
    }

    public void stopPeriodic() {
        _periodicThread.stop();
    }

    public void outputToDashboard() {
        SmartDashboard.putNumber("Periodic Manager DT", _dt);
    }
}
