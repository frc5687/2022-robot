/* Team 5687 (C)2022 */
package org.frc5687.rapidreact.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import org.frc5687.rapidreact.subsystems.DriveTrain;

public class Snap extends OutliersCommand {
    private DriveTrain _driveTrain;
    private Rotation2d _heading;

    public Snap(DriveTrain driveTrain, Rotation2d heading) {
        _driveTrain = driveTrain;
        _heading = heading;
    }

    @Override
    public void initialize() {
        _driveTrain.rotate(_heading);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        _driveTrain.setControlState(DriveTrain.ControlState.MANUAL);
        return true;
    }
}
