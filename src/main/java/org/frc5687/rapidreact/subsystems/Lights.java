package org.frc5687.rapidreact.subsystems;


import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.OutliersContainer;

import static org.frc5687.rapidreact.Constants.CANdle.*;

public class Lights extends OutliersSubsystem{
    private final CANdle _candle;
    private final CANdleConfiguration _config;
    private Animation _animate;
    private AnimationType _currentAnimation;
    private int[] _color;

    // subsystems
    private final Catapult _catapult;
    private final Climber _climber;
    private final Intake _intake;


    public Lights(OutliersContainer _container, Catapult catapult, Climber climber, Intake intake){
        super(_container);
        _catapult = catapult;
        _climber = climber;
        _intake = intake;
        _candle = new CANdle(RobotMap.CAN.CANDLE.PORT);
        _config = new CANdleConfiguration();
        //Set LED strip type
        _config.stripType = LEDStripType.RGB;
        //Sets LED brightness
        _config.brightnessScalar = Constants.CANdle.BRIGHTNESS;
        _candle.configAllSettings(_config);
        _color = GREEN;
    }

    //Set the color of the lights

    public void setColor(int[] color) {
        _color = color;
    }

    public int[] getColor() {
        return _color;
    }

    /**
     * Switch the current animation to the parameter.
     * @param animation
     */
    public void switchAnimation(AnimationType animation) {
        _currentAnimation = animation;
        switch (animation) {
            case COLOR_FLOW:
                _animate = new ColorFlowAnimation(
                        _color[0],
                        _color[1],
                        _color[2],
                        0,
                        SPEED,
                        NUM_LED,
                        ColorFlowAnimation.Direction.Forward);
                break;
            case FIRE:
                _animate = new FireAnimation(
                        BRIGHTNESS,
                        SPEED,
                        NUM_LED,
                        0.5,
                        0.5
                );
                break;
            case RAINBOW:
                _animate = new RainbowAnimation();
                break;
            case STROBE:
                _animate = new StrobeAnimation(
                        _color[0],
                        _color[1],
                        _color[2],
                        0,
                        SPEED,
                        NUM_LED
                );
                break;
            case STATIC:
                _animate = null;
                break;
        }
    }


    /**
     * Has all the logic for the lights, and updates the CANdle with animations and static colors.
     */
    @Override
    public void periodic() {
        // check if we are climbing.
        if (_climber.getStep().getValue() >= Climber.ClimberStep.PREP_TO_CLIMB.getValue()) {
            setColor(PURPLE);
        } else {
            if (_catapult.getAutomatShoot()) {
                switchAnimation(AnimationType.STATIC);
            } else {
                switchAnimation(AnimationType.STROBE);
            }
            if (_intake.isBallInCradle()) {
                if (_intake.isBallInItake()) {
                    setColor(GREEN);
                } else {
                    setColor(BLUE);
                }
            } else if (_intake.isBallInItake()) {
                setColor(YELLOW);
            } else {
                setColor(MAROON);
            }
        }

        if (_animate == null) {
            _candle.setLEDs(_color[0], _color[1], _color[2]);
        } else {
            _candle.animate(_animate);
        }
    }

    @Override
    public void updateDashboard() {

    }

    /**
     * Animation types
     */
    public enum AnimationType {
        COLOR_FLOW(0),
        FIRE(1),
        RAINBOW(2),
        STROBE(3),
        STATIC(4);

        private int _value;
        AnimationType(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }

}
