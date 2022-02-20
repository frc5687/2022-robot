/* Team 5687 (C)5687-2022 */
package org.frc5687.rapidreact.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.HallEffect;
import org.frc5687.rapidreact.util.OutliersContainer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import static org.frc5687.rapidreact.Constants.Climber.*;


public class Climber extends OutliersSubsystem{
    
    private final CANSparkMax _stationaryArm;
    private final RelativeEncoder _stationaryArmEncoder;

    private final CANSparkMax _rockerArm;
    private final RelativeEncoder _rockerArmEncoder;

    private final DoubleSolenoid _rocker;
    private final ProfiledPIDController _stationaryArmController;
    private final HallEffect _topHallEffect;
    private final HallEffect _bottomHallEffect;

    public Climber(OutliersContainer container) {
        super(container);
        _stationaryArm = new CANSparkMax(RobotMap.CAN.SPARKMAX.STATIONARY_CLIMBER, CANSparkMax.MotorType.kBrushless);
        _stationaryArm.setInverted(STATIONARY_ARM_REVERSED);
        _stationaryArm.setSmartCurrentLimit(STATIONARY_ARM_CURRENT_LIMIT);
        _stationaryArm.setIdleMode(IdleMode.kBrake);
        _stationaryArm.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20);
        _stationaryArm.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        _stationaryArm.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);

        _stationaryArmEncoder = _stationaryArm.getEncoder();

        _rockerArm = new CANSparkMax(RobotMap.CAN.SPARKMAX.ROCKER_CLIMBER, CANSparkMax.MotorType.kBrushless);
        _rockerArm.setInverted(ROCKER_ARM_REVERSED);
        _rockerArm.setSmartCurrentLimit(ROCKER_ARM_CURRENT_LIMIT);
        _rockerArm.setIdleMode(IdleMode.kBrake);
        _rockerArm.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20);
        _rockerArm.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        _rockerArm.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);

        _rockerArmEncoder = _rockerArm.getEncoder();

        _rocker = new DoubleSolenoid(
                PneumaticsModuleType.REVPH,
                RobotMap.PCM.CLIMBER_IN,
                RobotMap.PCM.CLIMBER_OUT
        );

        _topHallEffect = new HallEffect(RobotMap.DIO.STATIONARY_ARM_TOP_HALL);
        _bottomHallEffect = new HallEffect(RobotMap.DIO.STATIONARY_ARM_BOTTOM_HALL);


        _stationaryArmController = new ProfiledPIDController(
                kP,
                kI,
                kD,
                new TrapezoidProfile.Constraints(
                        MAX_VELOCITY_MPS,
                        MAX_ACCELERATION_MPSS
                )
        );
    }

    public void setStationaryArmSpeed(double speed) {
        _stationaryArm.set(speed);
    }

    // Revolution.
    public double getStationaryArmEncoderPosition() {
        return _stationaryArmEncoder.getPosition();
    }

    public double getStationaryArmPosition() {
        return (getStationaryArmEncoderPosition() / GEAR_REDUCTION) * WINCH_DRUM_CIRCUMFERENCE;
    }

    public double getStationaryArmControllerOutput(double goal) {
        return _stationaryArmController.calculate(goal, getStationaryArmPosition());
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