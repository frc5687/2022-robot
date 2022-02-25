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
    
    private CANSparkMax _stationaryArmWinch;
    private CANSparkMax _rockerArmWinch;
    private DoubleSolenoid _rocker;
    private ProfiledPIDController _staController;
    private ProfiledPIDController _rockController;
    private RelativeEncoder _stationaryArmWinchEncoder;
    private RelativeEncoder _rockerArmWinchEncoder;
    private HallEffect _staArmUp;
    private HallEffect _staArmDown;
    private boolean _armForward = false;
    private ClimberState _state = ClimberState.UNKNOW;
    private ClimberStep _step = ClimberStep.UNKNOW;
    private double _goal = 0.0;

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
        STOW(0),
        PREP_TO_CLIMB(1),
        ATTACH_MID(2),
        ATTACH_HIGH(3),
        ATTACH_TRANSVERSAL(4),
        UNKNOW(2);

        private final int _value;
        ClimberStep(int value) { 
            _value = value; 
        }

        public int getValue() { 
            return _value; 
        }
    }

    public void setState(ClimberState state){
        //Sets the current state of the climber
        _state = state;
    }

    public void setStep(ClimberStep step){
        //Sets the current step of the climbing process
        _step = step;
    }

    public Climber(OutliersContainer container) {
        super(container);
        _stationaryArmWinch = new CANSparkMax(RobotMap.CAN.SPARKMAX.STATIONARY_CLIMBER, CANSparkMax.MotorType.kBrushless);
        _rockerArmWinch = new CANSparkMax(RobotMap.CAN.SPARKMAX.ROCKER_CLIMBER, CANSparkMax.MotorType.kBrushless);
        _rocker = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.PCH.CLIMBER_IN, RobotMap.PCH.CLIMBER_OUT);

        _stationaryArmWinch.setIdleMode(IdleMode.kCoast);
        _stationaryArmWinch.setInverted(Constants.Climber.STATIONARY_ARM_REVERSED);
        _staArmUp = new HallEffect(RobotMap.DIO.STATIONARY_ARM_TOP_HALL);
        _staArmDown = new HallEffect(RobotMap.DIO.STATIONARY_ARM_BOTTOM_HALL);
        _staController = new ProfiledPIDController(Constants.Climber.kP, Constants.Climber.kI, Constants.Climber.kD,  new TrapezoidProfile.Constraints(1.5, 2.0));
        _rockController = new ProfiledPIDController(Constants.Climber.kP, Constants.Climber.kI, Constants.Climber.kD, new TrapezoidProfile.Constraints(1.5, 2.0));
        _stationaryArmWinchEncoder = _stationaryArmWinch.getEncoder();
        _rockerArmWinchEncoder = _rockerArmWinch.getEncoder();
        _stationaryArmWinchEncoder.setPosition(0);
    }

    @Override
    public void periodic(){
        super.periodic();
        if(_stationaryArmWinch.getAppliedOutput() > Constants.Climber.STATIONARY_ARM_CURRENT_LIMIT){
            stop();
        }
    }

    public void setStaSpeed(double speed){
        //Sets the speed of SparkMax controlling the climber arm
        _stationaryArmWinch.set(speed);
    }

    public void stop(){
        _stationaryArmWinch.set(0.0);
        _stationaryArmWinch.setIdleMode(IdleMode.kBrake);
    }

    public void setStaGoal(double goal){
        //Sets the goal of the PID controller
        _goal = goal;
        _staController.setGoal(goal);
    }

    public double getStaPID(){
        //Get calculation form PID controller
        return _staController.calculate(_stationaryArmWinchEncoder.getPosition());
    }

    public void moveStaArm(ClimberState state, double position){
        //Move to 
        _state = state;
        setStaGoal(position);
        setStaSpeed(getStaPID());
    }

    public void dropDriveSpeed(){
        //Drop driveTrain speed 
        Constants.DriveTrain.MAX_MPS = Constants.Climber.CLIMB_DRIVE_SPEED;
    }

    public void setRockSpeed(double speed){
        //Sets the speed of SparkMax controlling the rocker arm
        _rockerArmWinch.set(speed);
    }

    public void setRockGoal(double goal){
        //Sets the goal of the PID controller
        _rockController.setGoal(goal);
    }

    public double getRockPID(){
        //Get calculation form PID controller
        return _rockController.calculate(_rockerArmWinchEncoder.getPosition());
    }

    public void moveRockArm(ClimberState state, double position){
        //Move to 
        _state = state;
        setStaGoal(position);
        setRockSpeed(getStaPID());
    }

    public void rockerIn(){
        //Rock arm in
        _rocker.set(Value.kForward);
    }

    public void rockerOut(){
        //Rock arm out
        _rocker.set(Value.kReverse);
    }

    public boolean isStaArmUp(){
        //Is the stationary arm up
        return _staArmUp.get();
    }

    public boolean isStaArmDown(){
        //Is the stationary arm down
        return _staArmDown.get();
    }

    public void zeroEncoder(){
        _stationaryArmWinchEncoder.setPosition(0);
    }

    public void invert(boolean inverted){
        _stationaryArmWinch.setInverted(inverted);
    }

    @Override
    public void updateDashboard() {
        metric("Stationary encoder", _stationaryArmWinchEncoder.getPosition());
        metric("PID goal", _goal);
        metric("PID calculation", _staController.calculate(_stationaryArmWinchEncoder.getPosition()));
        metric("State", _state.name());
        metric("Step", _step.name());
        metric("Stationary Arm Up", isStaArmUp());
        metric("Stationary Arm Down", isStaArmDown());
    }
}