/* Team 5687 (C)2022 */
package org.frc5687.rapidreact.util;

public interface OutlierPeriodic {
    /** Control processing periodic function */
    void controlPeriodic(double timestamp, double dt);

    /** Data processing periodic function */
    void dataPeriodic(double timestamp, double dt);
}
