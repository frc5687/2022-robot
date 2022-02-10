package org.frc5687.rapidreact.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.rapidreact.subsystems.Catapult;

public class Shoot extends SequentialCommandGroup {
    Shoot(Catapult catapult) {
        addCommands(new LowerCatapult(catapult),
                new TestSpring(catapult));
    }
}
