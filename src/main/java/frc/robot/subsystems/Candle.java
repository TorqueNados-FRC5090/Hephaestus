package frc.robot.subsystems;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.Constants.LEDConstants.LEDStrip;

public class Candle extends SubsystemBase {

    public CANdle candle;

    /** Creates a CANdle
     *  @param ID CAN ID of the CANdle
     */
    public Candle(){
        candle = new CANdle(999, "Default Name");
    }
    
    /** Sets the LEDs to the selected color
     *  @param color The {@link LEDColor} to use
     */
    public void setAll(LEDStrip strip, LEDColor color) {
        SolidColor req = new SolidColor(strip.getStartingIndex(), strip.getEndingIndex());
        candle.setControl(req.withColor(color.getAsCTREColor()));
    }
}