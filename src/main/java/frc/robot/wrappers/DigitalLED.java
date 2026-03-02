package frc.robot.wrappers;

import edu.wpi.first.wpilibj.DigitalOutput;

/** Wraps {@link DigitalInput} to represent a laser detector. */ 
public class DigitalLED {
    private DigitalOutput LED;

     /** 
     * Constructs DigitalLED object 
     * 
     * @param port The number of the DIO output
     */
    public DigitalLED(int port){

        LED = new DigitalOutput(port);
    }

    // Gets the current value of the LED
    public boolean isOn(){ return LED.get();}
    public boolean isOff(){ return !LED.get();}
    public boolean get(){ return LED.get();}

    // Sets the value of the LED
    public void off(){ LED.set(false);}
    public void on(){ LED.set(true);}

    /** @param set Whether the led should be turned on */
    public void set(boolean set){ LED.set(set);}

    /** This sets the LED to the opposite of its current state */
    public void toggle(){
        set(!get());
    }
}
