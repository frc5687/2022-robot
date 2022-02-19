/* Team 5687 (C)5687-2022 */
package org.frc5687.rapidreact.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.HallEffect;
import org.frc5687.rapidreact.util.OutliersContainer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class Climber extends OutliersSubsystem{
    
    private CANSparkMax _stationaryArm;
    private CANSparkMax _rockerArm;
    private DoubleSolenoid _rocker;
    private ProfiledPIDController _climberControl;
    private HallEffect _topHallEffect;
    private HallEffect _bottomHallEffect;

    public Climber(OutliersContainer container) {
        super(container);
        _stationaryArm = new CANSparkMax(RobotMap.CAN.SPARKMAX.STATIONARY_CLIMBER, CANSparkMax.MotorType.kBrushless);
        _rockerArm = new CANSparkMax(RobotMap.CAN.SPARKMAX.ROCKER_CLIMBER, CANSparkMax.MotorType.kBrushless);
        _rocker = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.PCM.CLIMBER_IN, RobotMap.PCM.CLIMBER_OUT);

        _stationaryArm.setIdleMode(IdleMode.kCoast);
        _stationaryArm.setInverted(false);
        _topHallEffect = new HallEffect(1);
        _bottomHallEffect = new HallEffect(2);
        _climberControl = new ProfiledPIDController(Constants.Climber.kP, Constants.Climber.kI, Constants.Climber.kD,  new TrapezoidProfile.Constraints(2.0, 6.2));
    }

    public void raiseSationaryArm(){
        //Raise right arm
        _stationaryArm.set(0.2);
    }

    public void climb(){
        _stationaryArm.set(-0.2);
    }

    public boolean isArmUp(){
        return _topHallEffect.get();
    }

    public boolean isArmDown(){
        return _bottomHallEffect.get();
    }

    public void stopClimb(){
        //Only for debuging
        _stationaryArm.set(0.0);
    }

    public void rock(){
        //Use the rocker
        _rocker.set(Value.kForward);
    }

    @Override
    public void updateDashboard() {
        metric("Stationary climber motor encoder", _stationaryArm.getEncoder().toString());
        metric("Stationary climber motor temp", _stationaryArm.getMotorTemperature());
    }
}