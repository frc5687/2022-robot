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
    private ProfiledPIDController _climberController;
    private HallEffect _staArmUp;
    private HallEffect _staArmDown;
    private boolean _armForward = false;

    public Climber(OutliersContainer container) {
        super(container);
        _stationaryArm = new CANSparkMax(RobotMap.CAN.SPARKMAX.STATIONARY_CLIMBER, CANSparkMax.MotorType.kBrushless);
        _rockerArm = new CANSparkMax(RobotMap.CAN.SPARKMAX.ROCKER_CLIMBER, CANSparkMax.MotorType.kBrushless);
        _rocker = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.PCM.CLIMBER_IN, RobotMap.PCM.CLIMBER_OUT);

        _stationaryArm.setIdleMode(IdleMode.kCoast);
        _stationaryArm.setInverted(false);
        _staArmUp = new HallEffect(8);
        _staArmDown = new HallEffect(6);
        _climberController = new ProfiledPIDController(Constants.Climber.kP, Constants.Climber.kI, Constants.Climber.kD,  new TrapezoidProfile.Constraints(1.5, 2.0));
        _climberController.setTolerance(0.5);
    }

    public void raiseRockerArm(){
        //Raise the rocker arm
        _rockerArm.setInverted(false);
        _rockerArm.set(0.6);
    }

    public void raiseSationaryArm(){
        //Raise right arm
        _stationaryArm.setInverted(false);
        _stationaryArm.set(0.6);
    }

    public void climb(){
        //Lower right arm
        _stationaryArm.setInverted(true);
        _stationaryArm.set(0.4);
    }

    public boolean isStaArmUp(){
        return _staArmUp.get();
    }

    public boolean isStaArmDown(){
        return _staArmDown.get();
    }

    public void stopClimb(){
        //Only for debuging
        _stationaryArm.set(0.0);
        _rockerArm.set(0.0);
    }

    public void forward(){
        //Use the rocker
        _rocker.set(Value.kForward);
        _armForward = true;
    }

    public void reverse(){
        //Use the rocker
        _rocker.set(Value.kReverse);
        _armForward = false;
    }

    public boolean isArmForward(){
        return _armForward;
    }

    @Override
    public void updateDashboard() {
        metric("Stationary climber motor encoder", _stationaryArm.getEncoder().toString());
        metric("Stationary climber motor temp", _stationaryArm.getMotorTemperature());
        metric("Arm Forward", isArmForward());
        metric("Arm Output", _stationaryArm.getAppliedOutput());
    }
}