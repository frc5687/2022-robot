/* Team 5687 (C)2022 */
package org.frc5687.rapidreact.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.subsystems.DriveTrain;

public class AutoAlign extends PIDCommand {

    public AutoAlign(DriveTrain drivetrain, double angle) {
        super(
                new PIDController(
                        Constants.DriveTrain.SNAP_kP,
                        Constants.DriveTrain.SNAP_kI,
                        Constants.DriveTrain.SNAP_kD),
                drivetrain::getYaw,
                angle,
                output -> drivetrain.drive(0, 0, output),
                drivetrain);

        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(1.0);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
