/* Team 5687 (C)5687-2022 */
package org.frc5687.rapidreact.subsystems;

import com.revrobotics.CANSparkMax;

import org.frc5687.rapidreact.util.OutliersContainer;


public class Climber extends OutliersSubsystem{
    
    private CANSparkMax _leftArm;
    private CANSparkMax _rightArm;

    public Climber(OutliersContainer container) {
        super(container);
        _leftArm = new CANSparkMax(4, CANSparkMax.MotorType.kBrushless);
        _rightArm = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
    }

    
    @Override
    public void updateDashboard() {

    }
}