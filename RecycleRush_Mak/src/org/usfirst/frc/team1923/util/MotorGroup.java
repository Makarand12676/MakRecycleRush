package org.usfirst.frc.team1923.util;

import edu.wpi.first.wpilibj.SpeedController;
import java.util.Vector;

/**
 * A group of Motors/Speedcontrollers
 * 
 * @author Pavan Hegde  
 * @version 1.1
 * @since Jan 20, 2014
 */
public class MotorGroup implements SpeedController{
    private Vector motors = new Vector();               //A Vector to store all of the speed controllers
    
    /**
     * Constructor to create a MotorGroup of 1 motor
     * @param controllerOne The Speed controller for the 1st motor
     */
    public MotorGroup(SpeedController controllerOne) {
        this(controllerOne, null, null, null);
    }
    
    /**
     * Constructor that creates a MotorGroup with 2 motors
     * @param controllerOne The controller for the first motor
     * @param controllerTwo The controller for the second controller
     */
    public MotorGroup(SpeedController controllerOne, SpeedController controllerTwo) {
        this(controllerOne, controllerTwo, null, null);
    }
    
    /**
     * Constructor that creates a MotorGroup with 3 Motors
     * @param controllerOne The controller for teh first Motor
     * @param controllerTwo The controller for the second motor
     * @param controllerThree  The controller for the third motor
     */
    public MotorGroup(SpeedController controllerOne, SpeedController controllerTwo, SpeedController controllerThree) {
        this(controllerOne, controllerTwo, controllerThree, null);
    }
    
    /**
     * Constructor that creates a MotorGroup with four motors
     * @param controllerOne The controller for the first motor
     * @param controllerTwo the controller for the second motor
     * @param controllerThree the controller for the third motor
     * @param controllerFour  the controller for the fourth motor
     */
    public MotorGroup(SpeedController controllerOne, SpeedController controllerTwo, SpeedController controllerThree, SpeedController controllerFour) {
        if (controllerOne != null) {
            motors.addElement(controllerOne);
        }
        if (controllerTwo != null) {
            motors.addElement(controllerTwo);
        }
        if (controllerThree != null) {
            motors.addElement(controllerThree);
        }
        if (controllerFour != null) {
            motors.addElement(controllerFour);
        }
    }
    
    /**
     * Disables each motor in the MotorGroup
     */
    public void disable() {
        for (int i = 0; i < motors.size(); i++) {((SpeedController) motors.elementAt(i)).disable(); }
    }
    
    /**
     * Sets the speed of each motor in the MotorGroup
     * @param speed  the desired speed
     */
    public void set(double speed) {
        for (int i = 0; i < motors.size(); i++) {((SpeedController) motors.elementAt(i)).set(speed); }
    }
    
    /**
     *  Sets the speed of each motor in the MotorGroup
     * @param speed the desired speed
     * @param syncGroup 
     */
    public void set(double speed, byte syncGroup) {
        for (int i = 0; i < motors.size(); i++) {((SpeedController) motors.elementAt(i)).set(speed, syncGroup); }
    }
    
    /**
     * @return a double representing the SpeedController
     */
    public double get() {
      return ((SpeedController) motors.elementAt(0)).get();
    }
    
    /**
     * @param output ... ...For use in PID loop
     */
    public void pidWrite(double output) {
        for (int i = 0; i < this.motors.size(); i++) {
            ((SpeedController) (this.motors.elementAt(i))).pidWrite(output);
        }
    }
}