package org.frc5687.rapidreact.util;

import org.frc5687.rapidreact.RobotMap;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PCH {

    private Compressor _comp;  
    
    public PCH(){
        _comp = new Compressor(RobotMap.PCM.COMP, PneumaticsModuleType.REVPH);
        _comp.enableDigital();
    }

    /**
     * Check to see if the compressor is enabled
     * @return
     */
    public boolean isEnabled(){
        return _comp.enabled();
    }

    /**
     * Get the pressure in PSI
     * @return
     */
    public double getPressurePSI(){
        return _comp.getPressure();
    }

    /**
     * Gets the current in amps
     * @return
     */
    public double getCurrentAmps(){
        return _comp.getCurrent();
    }
}
