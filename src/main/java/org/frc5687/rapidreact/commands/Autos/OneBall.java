package org.frc5687.rapidreact.commands.Autos;

import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OneBall extends SequentialCommandGroup{

    //_driveTrain, _catapult, _intake, _oi
    public OneBall(DriveTrain driveTrain, Catapult catapult, Intake intake, OI oi){
        addCommands(new ReleaseArm(catapult), new DriveForTime(driveTrain, 2000, true));
    }
}
