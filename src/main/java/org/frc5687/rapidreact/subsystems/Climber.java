/* Team 5687 (C)5687-2022 */
package org.frc5687.rapidreact.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
    private RelativeEncoder _stationaryArmWinch;
    private HallEffect _staArmUp;
    private HallEffect _staArmDown;
    private boolean _armForward = false;
    private ClimberState _state = ClimberState.UNKNOW;
    private ClimberStep _step = ClimberStep.UNKNOW;

    public enum ClimberState {
        KILL(1),
        RETRACTING_S_ARM(2),
        RETRACTING_R_ARM(3),
        EXTENDING_S_ARM(4),
        EXTENDING_R_ARM(5),
        ROCKING(6),
        UNKNOW(0);

        private final int _value;
        ClimberState(int value) { 
            _value = value; 
        }

        public int getValue() { 
            return _value; 
        }
    }

    public enum ClimberStep{
        FIRST_STEP(0),
        SECOND_STEP(1),
        UNKNOW(2);

        private final int _value;
        ClimberStep(int value) { 
            _value = value; 
        }

        public int getValue() { 
            return _value; 
        }
    }

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
        _climberController.setTolerance(0.1);
        _stationaryArmWinch = _stationaryArm.getEncoder();
    }

    @Override
    public void periodic(){
        super.periodic();
    }

    public void setState(ClimberState state){
        //Sets the current climber state
        _state = state;
    }

    public void setStep(ClimberStep step){
        //Sets the current climber step
        _step = step;
    }

    public String getStateName(){
        //Returns current state as a string
        return _state.name();
    }

    public String getStepName(){
        //Returns current step as a string
        return _step.name();
    }

    public void setWinchGoal(double winchGoal){
        //Set to climber PID controllers goal
        _climberController.setGoal(winchGoal);
    }

    public double getWinchPosition(){
        //Get the current position of the climber winch
        return _stationaryArmWinch.getPosition();
    }

    public void setStationaryArmSpeed(double speed){
        //Set the stationary arm speed
        _stationaryArm.set(speed);
    }

    public void setRockerArmSpeed(double speed){
        //Set the speed of the rocker arm
        _rockerArm.set(speed);
    }

    public double calculateWinch(){
        return _climberController.calculate(getWinchPosition());
    }

    public void retractStationaryArm(double setpoint){
        //Retract stationary arm
        setState(ClimberState.RETRACTING_S_ARM);
        setWinchGoal(setpoint);
        setStationaryArmSpeed(calculateWinch());
    }

    public void retractRockerArm(double setpoint){
        //Retract rocker arm
        setState(ClimberState.RETRACTING_R_ARM);
        setWinchGoal(setpoint);
        setRockerArmSpeed(calculateWinch());
    }

    public void extendStationaryArm(double setpoint){
        //Extend stationary arm
        setState(ClimberState.EXTENDING_S_ARM);
        setWinchGoal(setpoint);
        setStationaryArmSpeed(calculateWinch());
    }

    
    public void extendRockerArm(double setpoint){
        //Retract rocker arm
        setState(ClimberState.EXTENDING_R_ARM);
        setWinchGoal(setpoint);
        setRockerArmSpeed(calculateWinch());
    }

    public boolean atGoal(){
        //Is the PID controller at the set point
        return _climberController.atGoal();
    }

    public boolean isStaArmUp(){
        //Is the stationary arm up
        return _staArmUp.get();
    }

    public boolean isStaArmDown(){
        //Is the stationary arm down
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
        //Is the rocker arm forward
        return _armForward;
    }

    @Override
    public void updateDashboard() {
        metric("Stationary climber motor encoder", _stationaryArm.getEncoder().toString());
        metric("Stationary climber motor temp", _stationaryArm.getMotorTemperature());
        metric("Arm Forward", isArmForward());
        metric("Arm Output", _stationaryArm.getAppliedOutput());
        metric("Winch position", getWinchPosition());
        metric("PID output", calculateWinch());
        metric("Climber State", getStateName());
        metric("Climber Step", getStepName());
    }
}