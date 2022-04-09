package org.frc5687.rapidreact.subsystems;


import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.OutliersContainer;

public class Lights extends OutliersSubsystem{

    private CANdle _candle;
    private CANdleConfiguration _config;

    public Lights(OutliersContainer _container){
        super(_container);
        _candle = new CANdle(RobotMap.CAN.CANDLE.PORT);
        _config = new CANdleConfiguration();
        //Set LED strip type
        _config.stripType = LEDStripType.RGB;
        //Sets LED brightness
        _config.brightnessScalar = Constants.CANdle.BRIGHTNESS;
        _candle.configAllSettings(_config);

        _candle.setLEDs(255, 255, 255);
    }

    //Set the color of the lights


    public void pink(){
        _candle.setLEDs(255, 105, 18);
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
