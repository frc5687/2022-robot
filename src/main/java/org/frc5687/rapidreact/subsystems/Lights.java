package org.frc5687.rapidreact.subsystems;


import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.OutliersContainer;

public class Lights extends OutliersSubsystem{

    private CANdle _candle;
    private CANdleConfiguration _config;
    private RainbowAnimation _rainbow;
    private StrobeAnimation _strobe;

    public Lights(OutliersContainer _container){
        super(_container);
        _candle = new CANdle(RobotMap.CAN.CANDLE.PORT);
        _config = new CANdleConfiguration();
        //Set LED strip type
        _config.stripType = LEDStripType.RGB;
        //Sets LED brightness
        _config.brightnessScalar = Constants.CANdle.BRIGHTNESS;
        _candle.configAllSettings(_config);

        //_candle.setLEDs(255, 255, 255);
        _rainbow = new RainbowAnimation(1, 0.5, 64);
    }

    //Set the color of the lights

    /**
     * Sets a rainbow animation
     */
    public void rainbow(){
        _candle.animate(_rainbow);
    }

    /**
     * Sets a strobe a animation
     */
    public void strobe(int[] color){
        _strobe = new StrobeAnimation(color[0], color[1], color[2]);
        _candle.animate(_strobe);
    }

    public void setPink(){
        _candle.setLEDs(255, 105, 18);
    }

    public void setYellow(){
        _candle.setLEDs(255, 255, 0);
    }

    public void setWhite(){
        _candle.setLEDs(0, 0, 0);
    }

    public void setRed(){
        _candle.setLEDs(255, 0, 0);
    }

    public void setGreen(){
        _candle.setLEDs(0, 255, 0);
    }

    public void setBlue(){
        _candle.setLEDs(0, 0, 255);
    }

    public void setGold(){
        _candle.setLEDs(255, 215, 0);
    }

    public void setPurple(){
        _candle.setLEDs(160, 32, 240);
    }

    @Override
    public void updateDashboard() {

    }
}
