/* Team 5687 (C)2022 */
package org.frc5687.rapidreact.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotContainer;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.commands.Climber.AttachHighRungCommand.Step;
import org.frc5687.rapidreact.util.HallEffect;
import org.frc5687.rapidreact.util.OutliersContainer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The Climber subsytem consists of two hooked arms on spring-loaded cylinders, restrained by winches.
 * Paying the winch out allows the hook to extend, retracting the winch pulls the hook down.
 * One arm is stationary, the other rocks foreward and back driven by a pneumatic piston.
 * 
 * The winches are driven by baby neos on sparkmax controllers (with integral encoders) and the each arm
 * has a pair of hall-effect sensors to indicate when they are UP or DOWN.
 * 
 * The winches are controlled by ProfiledPIDControllers, with goals set using setStaGoal and setRockGoal.
 * The controllers are enabled/disabled using the enable
 */
public class Climber extends OutliersSubsystem{
    
    private TalonFX _stationaryArmWinch;
    private TalonFX _rockerArmWinch;
    private DoubleSolenoid _rocker;

    private HallEffect _staArmDown;
    private HallEffect _rockArmDown;
    private ClimberStep _step = ClimberStep.UNKNOWN;
    private boolean _climberStopped = true;
    private Lights _lights;

    private boolean _staControllerEnabled = false;
    private boolean _rockControllerEnabled = false;

    private double _staSpeed = 0.0;
    private double _rockSpeed = 0.0;

    private double _staGoal = 0.0;
    private double _rockGoal = 0.0;

    private DriveTrain _driveTrain;

    public void setStep(ClimberStep step){
        //Sets the current step of the climbing process
        _step = step;

        // If we are not STOWING set the drivetrain speed limit
        switch(_step) {
            case UNKNOWN:
            case STOW:
            case STOWED:
                _driveTrain.dropDriveSpeed(false);
            default:
                _driveTrain.dropDriveSpeed(true);
        }
    }
 
    public ClimberStep getStep() {
        return _step;
    }
    
    public Climber(OutliersContainer container, DriveTrain driveTrain, Lights lights) {
        super(container);
        _driveTrain = driveTrain;
        _lights = lights;


//        logMetrics("Stationary/Position", "Stationary/Goal", "Stationary/Enabled", "Stationary/Speed", "Stationary/Up", "Stationary/Up", "Rocker/Position", "Rocker/Goal", "Rocker/Enabled", "Rocker/Speed", "Rocker/Up", "Rocker/Down", "Rocker Cylinder");

        _stationaryArmWinch = new TalonFX(RobotMap.CAN.TALONFX.STATIONARY_CLIMBER);
        _stationaryArmWinch.setNeutralMode(NeutralMode.Brake);
        _stationaryArmWinch.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.Climber.ARM_CURRENT_LIMIT, Constants.Climber.CURRENT_THRES, Constants.Climber.ARM_THRES_TIME));
        _stationaryArmWinch.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
//        _stationaryArmWinch.setSensorPhase(Constants.Climber.STATIONARY_ENCODER_REVERSED);
        _stationaryArmWinch.setInverted(Constants.Climber.STATIONARY_ARM_REVERSED);

//        _stationaryArmWinch.configForwardSoftLimitThreshold((int)(Constants.Climber.STATIONARY_EXTENDED_METERS/Constants.Climber.TICKS_TO_METERS), 30);
//        _stationaryArmWinch.configForwardSoftLimitEnable(true, 30);
//        _stationaryArmWinch.configReverseSoftLimitThreshold((int) (Constants.Climber.STATIONARY_RETRACTED_METERS/Constants.Climber.TICKS_TO_METERS), 30);
//        _stationaryArmWinch.configReverseSoftLimitEnable(true, 30);

        _stationaryArmWinch.configMotionCruiseVelocity(Constants.Climber.CRUISE_VELOCITY);
        _stationaryArmWinch.configMotionAcceleration(Constants.Climber.ACCELERATION);
        _stationaryArmWinch.configVoltageMeasurementFilter(8);
        _stationaryArmWinch.enableVoltageCompensation(true);
        _stationaryArmWinch.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10,20);
        _stationaryArmWinch.configClosedloopRamp(0,50);

 
        _rockerArmWinch = new TalonFX(RobotMap.CAN.TALONFX.ROCKER_CLIMBER);
        _rockerArmWinch.setNeutralMode(NeutralMode.Brake);
        _rockerArmWinch.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,Constants.Climber.ARM_CURRENT_LIMIT, Constants.Climber.CURRENT_THRES, Constants.Climber.ARM_THRES_TIME));
        _rockerArmWinch.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
//        _rockerArmWinch.setSensorPhase(Constants.Climber.ROCKER_ENCODER_REVERSED);
        _rockerArmWinch.setInverted(Constants.Climber.ROCKER_ARM_REVERSED);

//        _rockerArmWinch.configForwardSoftLimitThreshold((int)(Constants.Climber.ROCKER_EXTENDED_METERS/Constants.Climber.TICKS_TO_METERS), 30);
//        _rockerArmWinch.configForwardSoftLimitEnable(true, 30);
//        _rockerArmWinch.configReverseSoftLimitThreshold((int) (Constants.Climber.ROCKER_RETRACTED_METERS/Constants.Climber.TICKS_TO_METERS), 30);
//        _rockerArmWinch.configReverseSoftLimitEnable(true, 30);

        _rockerArmWinch.configMotionCruiseVelocity(Constants.Climber.CRUISE_VELOCITY);
        _rockerArmWinch.configMotionAcceleration(Constants.Climber.ACCELERATION);
        _rockerArmWinch.configVoltageMeasurementFilter(8);
        _rockerArmWinch.enableVoltageCompensation(true);
        _rockerArmWinch.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10,20);
        _rockerArmWinch.configClosedloopRamp(0,50);

        _rocker = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.PCH.CLIMBER_IN, RobotMap.PCH.CLIMBER_OUT);

        //_staArmUp = new HallEffect(RobotMap.DIO.STATIONARY_ARM_TOP_HALL);
        _staArmDown = new HallEffect(RobotMap.DIO.STATIONARY_ARM_BOTTOM_HALL);

        //_rockArmUp = new HallEffect(RobotMap.DIO.ROCKER_ARM_TOP_HALL);
        _rockArmDown = new HallEffect(RobotMap.DIO.ROCKER_ARM_BOTTOM_HALL);

        // Setup PID stuff
        _stationaryArmWinch.config_kP(Constants.Climber.MOTION_MAGIC_SLOT, Constants.Climber.kP);
        _stationaryArmWinch.config_kI(Constants.Climber.MOTION_MAGIC_SLOT, Constants.Climber.kI);
        _stationaryArmWinch.config_kD(Constants.Climber.MOTION_MAGIC_SLOT, Constants.Climber.kD);
        _stationaryArmWinch.config_kF(Constants.Climber.MOTION_MAGIC_SLOT, Constants.Climber.kF);

        _rockerArmWinch.config_kP(Constants.Climber.MOTION_MAGIC_SLOT, Constants.Climber.kP);
        _rockerArmWinch.config_kI(Constants.Climber.MOTION_MAGIC_SLOT, Constants.Climber.kI);
        _rockerArmWinch.config_kD(Constants.Climber.MOTION_MAGIC_SLOT, Constants.Climber.kD);
        _rockerArmWinch.config_kF(Constants.Climber.MOTION_MAGIC_SLOT, Constants.Climber.kF);
    }

    /**
     * Directly sets the speed of the stationary arm winch.
     * Note that this is intended to be called only by the climber's controllers. 
     * @param speed
     */

    public void setStaSpeed(double speed) {
        _staSpeed = speed;
        _stationaryArmWinch.set(TalonFXControlMode.PercentOutput, _staSpeed);
    }

    /**
     * Directly sets the speed of the rocker arm winch.
     * Note that this is intended to be called only by the climber's controllers. 
     * @param speed
     */
    public void setRockSpeed(double speed) {
        _rockSpeed = speed;
        //_rockerArmWinch.set(TalonFXControlMode.MotionMagic, speed);
        _rockerArmWinch.set(TalonFXControlMode.PercentOutput, _rockSpeed);
            
    }


    /**
     * Disables both PID controllers and sets both winch motors to brake mode.
     */
    public void stop(){
        stopRockerArm();
        stopStationaryArm();
    }

    /**
     * Stops the stationary arm by disabling the PID controller, setting the winch speed to 0, and setting brake mode.
     */
    public void stopStationaryArm() {
        info("Stopping StationaryArm");
        _staControllerEnabled = false;
        setStaSpeed(0.0);
//        _stationaryArmWinch.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Stops the rocker arm by disabling the PID controller, setting the winch speed to 0, and setting brake mode.
     */
    public void stopRockerArm() {
        info("Stopping RockerArm");
        _rockControllerEnabled = false;
        setRockSpeed(0.0);
//        _rockerArmWinch.setNeutralMode(NeutralMode.Coast);
    }

    public void setClimbingLights(){
        _lights.setGreen();
    }

    /**
     * Sets the goal of the stationary arm PID controller
     * @param goalMeters in inches
     */
    public void setStaGoalMeters(double goalMeters){
        _staGoal = goalMeters;
        info("Setting StationaryArm goal to " + _staGoal);
        _stationaryArmWinch.set(ControlMode.MotionMagic, (_staGoal / Constants.Climber.TICKS_TO_METERS));
        _staControllerEnabled = true;
    }


    /**
     * Sets the goal of the rocker arm PID controller
     * @param goalMeters in inches
     */
    public void setRockGoalMeters(double goalMeters){
        _rockGoal = goalMeters;
        info("Setting RockerArm goal to " + _rockGoal);
        //Sets the goal of the PID controller
        _rockerArmWinch.set(ControlMode.MotionMagic, (_rockGoal / Constants.Climber.TICKS_TO_METERS));
        _rockControllerEnabled = true;
    }


    /**
     * Gets the position of the stationary arm in meters
     * @return
     */
    public double getStaPositionMeters(){
        return _stationaryArmWinch.getSelectedSensorPosition() * Constants.Climber.TICKS_TO_METERS;
    }

    /**
     * Get the rocker arm position in meters
     * @return
     */
    public double getRockPositionMeters(){
        return _rockerArmWinch.getSelectedSensorPosition() * Constants.Climber.TICKS_TO_METERS;
    }

    /**
     * Run the winch PID controllers.  Called once per cycle by the various Climber Command classes.
     * Using the rocker arm and the stationary arms encoders with the string to get the distance
     * the arm is extened, that is used as the input to the PID controllers
     */
    public void runControllers() {
        if (!_staControllerEnabled) {
            setStaSpeed(0.0);
        }
        if (!_rockControllerEnabled) {
            setRockSpeed(0.0);
        }
    }

    /**
     * Pushes the rocker-arm piston in.
     */
    public void rockerIn(){
        //Rock arm in
        info("Setting Rocker in");
        _rocker.set(Value.kForward);
    }

    /**
     * Pushes the rocker-arm piston out.
     */
    public void rockerOut(){
        info("Setting Rocker out");
        //Rock arm out
        _rocker.set(Value.kReverse);
    }

    public void dropDriveSpeed(boolean value){
        //Drop driveTrain speed
        _driveTrain.dropDriveSpeed(value); 
    }

    /**
     * Checks the stationary arm "down" hall-effect sensor.
     * @return true if the sensor is triggered or if the arm is stalled
     */
    public boolean isStaArmDown(){
        return _staArmDown.get() || isStaArmStalled();
    }

    private int _staStallCycles = 0;
    private int _rockStallCycles = 0;

    /**
     * Checks to see if the stationary arm motor is stalled.
     * It is stalled if the amperage draw is > Constant and it has not moved in more than Constant cycles
     */
    public boolean isStaArmStalled() {
        if (isArmStalled(_stationaryArmWinch)) {
            _staStallCycles++;
        } else {
            _staStallCycles=0;
        }
        return _staStallCycles > Constants.Climber.MAX_STALL_CYCLES;
    }

    /**
     * Checks to see if the rocker arm motor is stalled.
     * It is stalled if the amperage draw is > STALL_CURRENT and it has not moved in more than MAX_STALL_CYCLES cycles
     */
    public boolean isRockArmStalled() {
        if (isArmStalled(_rockerArmWinch)) {
            _rockStallCycles++;
        } else {
            _rockStallCycles=0;
        }

        metric("Rocker/Current", _rockerArmWinch.getSupplyCurrent());
        metric("Rocker/StallCycles", _rockStallCycles);
        metric("Rocker/Velocity", _rockerArmWinch.getSelectedSensorVelocity());


        info("Rocker current " +_rockerArmWinch.getSupplyCurrent() + "  rocker velocity " +  _rockerArmWinch.getSelectedSensorVelocity() + " rocker cycles " + _rockStallCycles);
        return _rockStallCycles > Constants.Climber.MAX_STALL_CYCLES;
    }

    /** 
     * Checks to see if an arm is stalling.  An arm is stalling if the controller is sending more than STALL_CURRENT current
     * but the encoder is reading less that STALL_MIN_RPM rotations.
     */
    private boolean isArmStalled(TalonFX controller) {
       return controller.getSupplyCurrent() > Constants.Climber.STALL_CURRENT
            && Math.abs(controller.getSelectedSensorVelocity()) < Constants.Climber.STALL_MIN_RPM;  
    }

    /**
     * Checks the rocker arm "down" hall-effect sensor.
     * @return true if the sensor is triggered or the arm motor stalled
     */
    public boolean isRockArmDown(){
        return _rockArmDown.get() || isRockArmStalled();
    }

    /**
     * Sets the stationary arm encoder to 0.  This should be called when the "down" HE is triggered.
     */
    public void zeroStationaryArmEncoder(){
        _stationaryArmWinch.setSelectedSensorPosition(0);
    }

    /**
     * Sets the rocker arm encoder to 0.  This should be called when the "down" HE is triggered.
     */
    public void zeroRockerArmEncoder() {
        _rockerArmWinch.setSelectedSensorPosition(0);
    }

    @Override
    public void updateDashboard() {
        metric("Stationary/Position", getStaPositionMeters());
        metric("Stationary/Goal", _staGoal);
        metric("Stationary/Enabled", _staControllerEnabled);
        metric("Stationary/Encoder", _stationaryArmWinch.getSelectedSensorPosition());
        metric("Stationary/Speed", _staSpeed); 
        metric("Stationary/Down", _staArmDown.get());
        metric("Stationary/Current", _stationaryArmWinch.getSupplyCurrent());
        metric("Stationary/StallCycles", _staStallCycles);
        metric("Stationary/Velocity", _stationaryArmWinch.getSelectedSensorVelocity());

        metric("Rocker/Position", getRockPositionMeters());
        metric("Rocker/Goal", _rockGoal);
        metric("Rocker/Enabled", _rockControllerEnabled);
        metric("Rocker/Speed", _rockSpeed);
        metric("Rocker/Encoder", _rockerArmWinch.getSelectedSensorPosition());
        metric("Rocker/Down", isRockArmDown());
        metric("Rocker Cylinder", getRockerLabel());

        metric("Rocker/Current", _rockerArmWinch.getSupplyCurrent());
        metric("Rocker/StallCycles", _rockStallCycles);
        metric("Rocker/Velocity", _rockerArmWinch.getSelectedSensorVelocity());

        metric("Step", _step.name());
        metric("Climber stopped", _climberStopped);
    }


    /**
     * Checks whether the stationary arm PID controlled is at its goal.
     * @return true if within tolerance
     */
    public boolean isStaAtGoal() {
        return Math.abs(getStaPositionMeters() - _staGoal) < Constants.Climber.TOLERANCE;
    }

    /**
     * Checks whether the rocker arm PID controlled is at its goal.
     * @return true if within tolerance
     */
    public boolean isRockAtGoal() {
        return Math.abs(getRockPositionMeters() - _rockGoal) < Constants.Climber.TOLERANCE;
    }


    /**
     * @return the current stationary arm winch speed.
     */
    public double getStaSpeed() {
        return _staSpeed;
    }


    /**
     * @return the current rocker arm winch speed.
     */
    public double getRockSpeed() {
        return _rockSpeed;
    }

    /**
     * @return the position (In or Out) of the rocker piston
     */
    public String getRockerLabel() {
        return(_rocker.get() == Value.kForward) ? "In" : "Out";
    }

    /**
     * Flips the Rocker to the other side.
     */
    public void rockerFlip(){
        info("Flipping Rocker");
        //Flippin' the rockah
        if(_rocker.get() == Value.kForward) {
            rockerOut();
        }else if(_rocker.get() == Value.kReverse) {
            rockerIn();
        }else{
            info("I dunno what rocker state this is, flipping rocker out.");
            rockerOut();

        }
    }


    public enum ClimberStep {
        UNKNOWN(0),
        STOW(1),
        STOWED(2),
        PREP_TO_CLIMB(3),
        READY_TO_CLIMB(4),
        RESET_TO_MID(5),
        ATTACH_MID(6),
        ATTACHED_MID(7),
        RESET_TO_HIGH(8),
        ATTACH_HIGH(9),
        ATTACHED_HIGH(10),
        RESET_TO_TRAVERSE(11),
        ATTACH_TRAVERSAL(12),
        ATTACHED_TRAVERSAL(13),
        DONE(14);

        private final int _value;
        ClimberStep(int value) { 
            _value = value; 
        }

        public int getValue() { 
            return _value; 
        }
    }

}