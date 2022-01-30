/* Team 5687 (C)5687-2022 */
package org.frc5687.rapidreact.subsystems;

import com.revrobotics.CANSparkMax;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.Robot;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.OutliersContainer;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class Climber extends OutliersSubsystem{
    
    private CANSparkMax _leftArm;
    private CANSparkMax _rightArm;
    private DoubleSolenoid _rocker;

    public Climber(OutliersContainer container) {
        super(container);
        _leftArm = new CANSparkMax(RobotMap.CAN.SPARKMAX.Climber.LEFT_ARM, CANSparkMax.MotorType.kBrushless);
        _rightArm = new CANSparkMax(RobotMap.CAN.SPARKMAX.Climber.RIGHT_ARM, CANSparkMax.MotorType.kBrushless);
        _rocker = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.PCM.Climber.ROCKER_HIGH, RobotMap.PCM.Climber.ROCKER_LOW);
    }

    public void raiseArms(){
        //Raise both arms
    }

    public void raiseLeftArm(){
        //Raise left arm
    }

    public void raiseRightArm(){
        //Raise right arm
    }

    public void rock(){
        //Use the rocker
        _rocker.set(Value.kForward);
    }

    @Override
    public void updateDashboard() {

    }
}