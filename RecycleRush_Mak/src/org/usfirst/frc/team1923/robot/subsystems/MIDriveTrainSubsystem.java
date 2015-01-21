package org.usfirst.frc.team1923.robot.subsystems;



import org.usfirst.frc.team1923.robot.RobotMap;
import org.usfirst.frc.team1923.robot.commands.DriveWithJoyStickCommand;
import org.usfirst.frc.team1923.util.MotorGroup;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class MIDriveTrainSubsystem extends Subsystem {
    
	// Put methods for controlling this subsystem
    // here. Call these from Commands.
	// distance per pulse of a drive-wheel encoder, in inches. [CHANGE THESE VALUES!!!!!!!!!!]
	// Declare variable for the robot drive system
    private RobotDrive robotDriveTrain = RobotMap.robotDriveTrain;
    private Timer timer;
    private double timeOut = 2.0;
    // Drive Wheel Encoders
    private Encoder driveEncoderLeft = RobotMap.driveEncoderLeft;
    private Encoder driveEncoderRight = RobotMap.driveEncoderRight;
    // gyro.
    private Gyro gyro = RobotMap.gyro;
   // private MotorGroup left = RobotMap.driveLeftSide;
   // private MotorGroup right = RobotMap.driveRightSide;
	
	
	private static final double NUM_CLICKS = 256, //distance per pulse = 0.0491"/pulse
    							GEAR_RATIO = 1.0/1.0, 
					            WHEEL_CIRCUMFERENCE = 12.56,   // 4 inches wheels
					            Pg = 0.1, Ig = 0.005, Dg = 0.0,     // LEAVE THESE CONSTANTS ALONE!
					            Pe = 0.5, Ie = 0.01, De = 0.0,      // LEAVE THESE CONSTANTS ALONE!
					            PID_LOOP_TIME = .05, 
					            gyroTOLERANCE = 2.0,            // 0.2778% error ~= 0.5 degrees...?
					            encoderTOLERANCE = 2.0;         // +/- 2" tolarance
	
	private PIDController gyroPID, leftEncPID, rightEncPID;//, encPID, accelPID;
	
	public MIDriveTrainSubsystem() {
        
		// Set distance per pulse for each encoder
        driveEncoderLeft.setDistancePerPulse(GEAR_RATIO*WHEEL_CIRCUMFERENCE/NUM_CLICKS);
        driveEncoderRight.setDistancePerPulse(GEAR_RATIO*WHEEL_CIRCUMFERENCE/NUM_CLICKS);
        // Set PID source parameter to Distance...
        driveEncoderLeft.setPIDSourceParameter(Encoder.PIDSourceParameter.kDistance);
        driveEncoderRight.setPIDSourceParameter(Encoder.PIDSourceParameter.kDistance);
        // Set Tolerances
        leftEncPID.setAbsoluteTolerance(encoderTOLERANCE);
        rightEncPID.setAbsoluteTolerance(encoderTOLERANCE);
        gyroPID.setAbsoluteTolerance(gyroTOLERANCE);
        
        // Reset the encoders.
        driveEncoderLeft.reset();
        driveEncoderRight.reset();
        gyro.reset();
                 
       // gyroPID = new PIDController(Pg,Ig,Dg,gyro,left,PID_LOOP_TIME);
        
             
       // leftEncPID = new PIDController(Pe,Ie,De,driveEncoderLeft,left,PID_LOOP_TIME);
       // rightEncPID = new PIDController(Pe,Ie,De,driveEncoderRight,right,PID_LOOP_TIME);
       
        // soft limits: 0 to 90 degrees...
        gyroPID.setInputRange(-90.0, 90.0);  
                

        leftEncPID.setInputRange(-1000.0,1000.0);       // adjust?
        rightEncPID.setInputRange(-1000.0,1000.0);       // adjust?
        
        
        gyroPID.setOutputRange(-0.7, 0.7);
        
        
        leftEncPID.setOutputRange(-1.0,1.0);
        rightEncPID.setOutputRange(-1.0,1.0);
       
        
        // Timer        
        timer = new Timer();
        timer.reset();
        timer.stop();
        
        
        
        
	}
   
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new DriveWithJoyStickCommand());
    }
    
    // Manual Drive 
    public void manualDrive(double x, double y) {
        disablePID();
    	//RobotMap.robotDriveTrain.tankDrive(x, y);
    	RobotMap.robotDriveTrain.tankDrive(x, y, true);
        
        
    }
    // Stop
    public void stop(){
    	//disablePID();
    	RobotMap.robotDriveTrain.tankDrive(0.0, 0.0);
    }
    
    // Drive Stright Distance Using Encoder
    public void driveStrightUsingEncoder(double dist,double maxTimeOut){
    	//TODO
    	gyroPID.disable();
        
        
        driveEncoderLeft.reset();
        driveEncoderRight.reset();
           
      
        if(!leftEncPID.isEnable()) leftEncPID.enable();
        if(!rightEncPID.isEnable()) rightEncPID.enable();
        
        leftEncPID.setSetpoint(dist);      // Check direction
        rightEncPID.setSetpoint(dist);     // Check direction
        
    	this.timeOut = maxTimeOut;
    	this.timer.reset();
    	this.timer.start();
    }
    
   
       
       
    public boolean reachedDistance(){
    	if (timer.get() > this.timeOut || (leftEncPID.onTarget() && rightEncPID.onTarget())){
    		disablePID();
    		timer.stop();
    		timer.reset();
    		return true;
    	}else{
    		return false;
    	}
    		
    	
    	
    }
    
    
    public double getLeftDistanceError(){
		   	
    	return leftEncPID.getError();
    	
    }
    
    public double getRightDistanceError(){
	   	
    	return rightEncPID.getError();
    	
    }
    
    /**
     *
     * @return Count from the encoder (since the last reset?).
     */
    public double getLeftEncoderCount() {
       return RobotMap.driveEncoderLeft.getRaw();
    }
    
    public double getRightEncoderCount() {
        return RobotMap.driveEncoderRight.getRaw();
     }
    
    /**
     *
     * @return Distance the encoder has recorded since the last reset, adjusted for the gear ratio.
     */
    public double getLeftEncoderDistance() {
        return RobotMap.driveEncoderLeft.getDistance();
    }
    
    public double getRightEncoderDistance() {
        return RobotMap.driveEncoderRight.getDistance();
    }
    
    public double getAvgEncoderDistance() {
        return (RobotMap.driveEncoderLeft.getDistance()+this.driveEncoderRight.getDistance())/2.0;
    }
    
    public double getSpeedDiff(){
    	return RobotMap.driveEncoderLeft.getRate() - RobotMap.driveEncoderRight.getRate();
    }
    
    // Gyro Base Turns
    public void turnRobotHeading(double angle){
    // TODO
    	
    }
    public void disablePID() {
        if(gyroPID.isEnable()) gyroPID.disable();
        
        if(leftEncPID.isEnable()) leftEncPID.disable();
        if(rightEncPID.isEnable()) rightEncPID.disable();
        
    }
    
}

