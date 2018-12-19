package org.usfirst.frc.team217.robot;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.Encoder;


//camera libraries
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

//I2C Libraries
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

//SPI
import edu.wpi.first.wpilibj.SPI;

//navX libary
import com.kauailabs.navx.frc.AHRS; 

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import edu.wpi.first.wpilibj.DigitalOutput;

import org.usfirst.frc.team217.robot.sensors.BNO055;

public class Robot extends IterativeRobot {

	/* talons for arcade drive */
	WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(4);
	WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(2);

	/* extra talons and victors for six motor drives */
	WPI_TalonSRX _leftSlave1 = new WPI_TalonSRX(3);
	WPI_TalonSRX _rightSlave1 = new WPI_TalonSRX(1);


	DifferentialDrive _drive = new DifferentialDrive(_frontLeftMotor, _frontRightMotor);

	Joystick _joy = new Joystick(0);
	Button button1 = new JoystickButton(_joy, 1),
	button2 = new JoystickButton(_joy, 2),
	button3 = new JoystickButton(_joy, 3),
	button4 = new JoystickButton(_joy, 4),
	button5 = new JoystickButton(_joy, 5),		
	button6 = new JoystickButton(_joy, 6),		
	button7 = new JoystickButton(_joy, 7),		
	button8 = new JoystickButton(_joy, 8),
	button9 = new JoystickButton(_joy, 9),
	button10 = new JoystickButton(_joy, 10),
	button11 = new JoystickButton(_joy, 11),
	button12 = new JoystickButton(_joy, 12);
	
	//Digital Output Creation
	DigitalOutput digitalTest = new DigitalOutput(12);

	//Compressor Creation
	Compressor compressor = new Compressor();
	
	DoubleSolenoid shiftSolenoid = new DoubleSolenoid(0,1);
	
	//Timer Creation
	Timer timer;

	//Counter Creation
	Count autonomousCounter = new Count();


	//Encoder Creation

	Encoder encR = new Encoder(1, 2, false, Encoder.EncodingType.k4X);
	Encoder encL = new Encoder(3, 4, false, Encoder.EncodingType.k4X);

	
	//Pixy Object Creation

	PixyExample testPixy = new PixyExample();
	//Thread ThreadPixy1 = new Thread(new PixyThread());
	//Thread ThreadPixy;
	//Thread ThreadPixy1;
	
	//BNO055 Creation
	private static BNO055 imu = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS, BNO055.vector_type_t.VECTOR_EULER);

	//navX Creation
	AHRS ahrs = new AHRS(SPI.Port.kMXP);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		/*
		 * take our extra talons and just have them follow the Talons updated in
		 * arcadeDrive
		 */
		_leftSlave1.follow(_frontLeftMotor);
		_rightSlave1.follow(_frontRightMotor);

		/* drive robot forward and make sure all 
		 * motors spin the correct way.
		 * Toggle booleans accordingly.... */
		_frontLeftMotor.setInverted(true);
		_leftSlave1.setInverted(true);

		
		_frontRightMotor.setInverted(true);
		_rightSlave1.setInverted(true);

		
		shiftSolenoid.set(DoubleSolenoid.Value.kOff);
		shiftSolenoid.set(DoubleSolenoid.Value.kForward);
		shiftSolenoid.set(DoubleSolenoid.Value.kReverse);

		//create timer
		 timer = new Timer();


		//Set Encoder Values
		
		encR.setMaxPeriod(.1);
		encL.setMaxPeriod(.1);
		encR.setMinRate(10);
		encL.setMinRate(10);
		encR.setDistancePerPulse(5);
		encL.setDistancePerPulse(5);
		encR.setReverseDirection(true);
		encL.setReverseDirection(true);
		encR.setSamplesToAverage(7);
		encL.setSamplesToAverage(7);

		//int autoCounter = 0;
		
		PixyExample testPixy = new PixyExample();
		
		//Thread ThreadPixy1 = new Thread(new PixyThread());
		


	}//end of robotInit




	public void message(){

		//driveEncoders();

		int countR;
		int countL;

		double distanceR;
		double distanceL;
		

		countR = encR.getRaw();
		countL = encL.getRaw();

		distanceR = encR.getDistance();
		distanceL = encL.getDistance();

		//SmartDashboard.putString("DB/String 0", "Hello World");

		//String arduinoMessage = messageI2C();


		System.out.println("EncR " + countR + " EncL " + countL); 

		
		//System.out.println(arduinoMessage);

	}

	
	public void controlShifter() {
		
		compressor.start();
		compressor.setClosedLoopControl(true);
		
		if (_joy.getRawButtonPressed(2) == true) {
			
			shiftSolenoid.set(DoubleSolenoid.Value.kForward);
			
		}
		
		else if (_joy.getRawButtonPressed(3) == true) {
			
			shiftSolenoid.set(DoubleSolenoid.Value.kReverse);
			
		}	
		
	}//end of controlShifter
	
//Begining of I2C Arduino Pixy Classes


	public String messageI2C(){

		//SmartDashboard.putString("DB/String 0", "Hello World");

		I2C Wire = new I2C(Port.kOnboard, 4);//uses the i2c port on the RoboRIO
														//uses address 4, must match arduino
	 	int MAX_BYTES = 32;
		
		

		byte[] data = new byte[MAX_BYTES];//create a byte array to hold the incoming data
		Wire.read(4, MAX_BYTES, data);//use address 4 on i2c and store it in data
 
		
		String output = new String(data);//create a string from the byte array
		int pt = output.indexOf((char)255);
		return (String) output.subSequence(0, pt < 0 ? 0 : pt);//im not sure what these last two lines do
		
		
		//System.out.println(data[0]+ ", " + data[1]+ ", "+ data[2]+"," + data[3]); 
		//System.out.println(data);
	}







public class PixyPacket {
	double x, y;
	double area;
	
	//for our use we only cared about a x, y, and area
	//you could get any data you wanted
}


public class M_I2C {

	//changed from private static to just private
	private I2C Wire = new I2C(Port.kOnboard, 4);//uses the i2c port on the RoboRIO
														//uses address 4, must match arduino
	private static final int MAX_BYTES = 32;
	
	public void write(String input){//writes to the arduino 
			char[] CharArray = input.toCharArray();//creates a char array from the input string
			byte[] WriteData = new byte[CharArray.length];//creates a byte array from the char array
			for (int i = 0; i < CharArray.length; i++) {//writes each byte to the arduino
				WriteData[i] = (byte) CharArray[i];//adds the char elements to the byte array 
			}
			Wire.transaction(WriteData, WriteData.length, null, 0);//sends each byte to arduino
			
	}
	
	
		private String read(){//function to read the data from arduino
		byte[] data = new byte[MAX_BYTES];//create a byte array to hold the incoming data
		Wire.read(4, MAX_BYTES, data);//use address 4 on i2c and store it in data
		String output = new String(data);//create a string from the byte array
		int pt = output.indexOf((char)255);
		return (String) output.subSequence(0, pt < 0 ? 0 : pt);//im not sure what these last two lines do
															   //sorry :(
	}
	
	
	
	public PixyPacket getPixy(){//reads the data from arduino and saves it
		String info[] = read().split("\\|");//everytime a "|" is used it splits the data,
											//and adds it as a new element in the array
		PixyPacket pkt = new PixyPacket();  //creates a new packet to hold the data 
		if(info[0].equals("none") || info[0].equals("")){//checks to make sure there is data 
			pkt.x = -1;//the x val will never be -1 so we can text later in code to make sure 
					   //there is data
			pkt.y = -1;
			pkt.area = -1;
		}else if(info.length == 3){//if there is an x, y, and area value the length equals 3
			pkt.x = Double.parseDouble(info[0]);//set x
			pkt.y = Double.parseDouble(info[1]);//set y
			pkt.area = Double.parseDouble(info[2]);//set area
		}
		return pkt;
	}//end of getPixy
	
 



	
	
	}//end of M_I2C


	
public class PixyExample {
	M_I2C i2c = new M_I2C();//setup the i2c interface
	I2C mxpWire = new I2C(I2C.Port.kMXP, 15);
	PixyPacket pkt = i2c.getPixy();//create a pixy packet to hold data
	String testPacket;
	String packet;
	int i2cTurnForward;
	double turnValue;
	int turnValueHolder;
	double calculateForwardPower;
	double forwardValue;
	int xPosition;
	
	//String[] packet = new String[];

/*
	
	public void centerOnObject(){
	if(pkt.x != -1){//if data is exist
		if(pkt.x < .48 || pkt.x > .52){
			//and the 'object', whatever it may be is not in the center
		    //the code on the arduino decides what object to send
			while(pkt.x < .48 || pkt.x > .52){//while it is not center
				
				if(pkt.x < .48){//if its on the left side of robot, turn left
				
					//drive.setLDrive(-0.2);//this is our left side of tank drive
					//drive.setRDrive(0.2);//you drive code might differ
					
				}
				if(pkt.x > .52){//if its on the right side of robot, turn right
					
					//drive.setLDrive(0.2);
					//drive.setRDrive(-0.2);
					
				}
				if(pkt.y == -1)//Restart if ball lost during turn
					break;
				pkt = i2c.getPixy();//refresh the data
				System.out.println("XPos: " + pkt.x);//print the data just to see
			}

	}
}
}//end centerOnObject
*/

public void mxpTest(){

		digitalTest.disablePWM();
		
		digitalTest.pulse(10.0);

		digitalTest.set(false);

mxpWire.write(15, 4);

}//end mxpTest


public void pixyTest(){

packet = i2c.read();

 try
    {
 i2cTurnForward = Integer.valueOf(packet.trim());
    }

catch (NumberFormatException nfe)
    {
      System.out.println("NumberFormatException: " + nfe.getMessage());
    }


System.out.println("XPos: " + packet);

	timer.reset(); // Resets the timer to 0
	timer.start(); // Start counting

turnValue = 0.0;


turnValueHolder = i2cTurnForward - ((i2cTurnForward / 100)) * 100;
calculateForwardPower = ((i2cTurnForward)/100);
calculateForwardPower = (calculateForwardPower/10);
//  turnValue = ((turnValueHolder * (2/11) / 10) + 0.35);

switch(turnValueHolder) {

case 0:
turnValue = 0.0;
forwardValue = 0.0;
break;
case 1:
turnValue = -0.55;
forwardValue = 0.0;
break;
case 2:
turnValue = -0.51;
forwardValue = 0.0;
break;
case 3:
turnValue = -0.47;
forwardValue = 0.0;
break;
case 4:
turnValue = -0.41;
forwardValue = 0.0;
break;
case 5:
turnValue = -0.39;
forwardValue = 0.0;
break;
case 6:
turnValue = 0.0;
forwardValue = calculateForwardPower;
break;
case 7:
turnValue = 0.39;
forwardValue = 0.0;
break;
case 8:
turnValue = 0.41;
forwardValue = 0.0;
break;
case 9:
turnValue = 0.47;
forwardValue = 0.0;
break;
case 10:
turnValue = 0.51;
forwardValue = 0.0;
break;
case 11:
turnValue = 0.55;
forwardValue = 0.0;
break;
default:
turnValue = 0.0;
forwardValue = 0.0;
break;

}

//forwardValue = i2cTurnForward/1000;


//System.out.println("turnValueHolder: " + turnValueHolder);
System.out.println("i2cTurnForward: " + i2cTurnForward);

 
timer.reset(); // Resets the timer to 0
timer.start(); // Start counting

while(timer.get() <= .12){  //was .17

_drive.arcadeDrive(forwardValue, turnValue);

}


/*
packet = i2c.read();

System.out.println("XPos: " + packet);

	timer.reset(); // Resets the timer to 0
	timer.start(); // Start counting

       switch (packet) {
            case "0": while(timer.get() <= .5){

					_drive.arcadeDrive(0.37, 0.0);

					}//end of case 1   
                     break;
            case "1": while(timer.get() <= .5){
				_drive.arcadeDrive(0.0, -0.37);
			}	
			break;
			case "2": while(timer.get() <= .5) {
				_drive.arcadeDrive(0.0, 0.37);
			}				
            break;
            default: _drive.arcadeDrive(0.0, 0.0);
                     break;
        }//End pixy switch
*/
/*
       switch (packet) {
            case "1": while(timer.get() <= .5){

					_drive.arcadeDrive(0.0, -0.6);

					}//end of case 1   
                     break;
            case "2": while(timer.get() <= .5){
				_drive.arcadeDrive(0.0, -0.5);
			}	
			break;
			case "3": while(timer.get() <= .5) {
				_drive.arcadeDrive(0.0, -0.4);
			}				
            break;
			case "4": while(timer.get() <= .5) {
				_drive.arcadeDrive(0.4, 0.0);
			}				
            break;
			case "5": while(timer.get() <= .5) {
				_drive.arcadeDrive(0.0, 0.4);
			}				
            break;
			case "6": while(timer.get() <= .5) {
				_drive.arcadeDrive(0.0, 0.5);
			}		
			break;		
			case "7": while(timer.get() <= .5) {
				_drive.arcadeDrive(0.0, 0.6);
			}				
            break;
			case "0": while(timer.get() <= .5){
				_drive.arcadeDrive(0.0, 0.0);
			}					
                     break;
   
            default: _drive.arcadeDrive(0.0, 0.0);
                     break;
        }//End pixy switch
*/
//kate stepped on my dinner!!

/*
       switch (packet) {
            case "1": while(timer.get() <= .5){

					_drive.arcadeDrive(0.0, -0.4);

					}//end of case 1   
                     break;
            case "2": while(timer.get() <= .5){
				_drive.arcadeDrive(0.0, 0.4);
			}					
                     break;
			case "0": while(timer.get() <= .5){
				_drive.arcadeDrive(0.0, 0.0);
			}					
                     break;
   
            default: _drive.arcadeDrive(0.0, 0.0);
                     break;
        }//End pixy switch
*/	


/*
if(xPosition != 0.0){
			//and the 'object', whatever it may be is not in the center
		    //the code on the arduino decides what object to send

	    	 //_drive.arcadeDrive(0.0, 0.0);  // stop robot
				

				//xPosition = i2c.read();

				if(xPosition == 1.0){//if its on the left side of robot, turn left
				
					//drive.setLDrive(-0.2);//this is our left side of tank drive
					//drive.setRDrive(0.2);//you drive code might differ

					_drive.arcadeDrive(0.0, -0.4);
					
				}
				if(xPosition == 2.0){//if its on the right side of robot, turn right
					
					//drive.setLDrive(0.2);
					//drive.setRDrive(-0.2);

					_drive.arcadeDrive(0.0, 0.4);
					
				}_joy.getY();
				i_joy.getY();on == 0.0)//Restart if ball lost during turn
				__joy.getY();adeDrive(0.0, 0.0);
				
				
				/_joy.getY();n = i2c.read();

			

		}
  */      
	/*
				packet = i2c.read().split("\\|");
				pkt.x = Double.parseDouble(packet[0]);
				pkt.y = Double.parseDouble(packet[1]);
		
				System.out.println("Red  XPos: " + packet[0] + " YPos: " + packet[1] );// pkt.xprint the data just to see
				
				//System.o_joy.getY();t.println("Blue XPos: " + packet[2] + " YPos: " + packet[3]);

				//System.o_joy.getY();t.println("Yellow XPos: " + packet[4] + " YPos: " + packet[5]);
		
			if(pkt.x < .48_joy.getY();|| pkt.x > .52){
			//and the 'obj_joy.getY();ct', whatever it may be is not in the center
		    //the code on _joy.getY();he arduino decides what object to send

	    	 _drive.arcade_joy.getY();rive(0.0, 0.0);  // stop robot

			while(pkt.x < .48 || pkt.x > .52){//while it is not center
				
				if(pkt.x < .48){//if its on the left side of robot, turn left
				
					//drive.setLDrive(-0.2);//this is our left side of tank drive
					//drive.setRDrive(0.2);//you drive code might differ

					_drive.arcadeDrive(0.0, -0.4);
					
				}
				if(pkt.x > .52){//if its on the right side of robot, turn right
					
					//drive.setLDrive(0.2);
					//drive.setRDrive(-0.2);

					_drive.arcadeDrive(0.0, 0.4);
					
				}
				if(pkt.y == -1)//Restart if ball lost during turn
				_drive.arcadeDrive(0.0, 0.0);
				
				packet = i2c.read().split("\\|");

				pkt.x = Double.parseDouble(packet[0]);
				pkt.y = Double.parseDouble(packet[1]);

				System.out.println("Red  XPos: " + packet[0] + " YPos: " + packet[1] );// pkt.xprint the data just to see
			}

			}


				String packet[] = null;
		*/			

				

}//end pixyTest




}//end PixyExample


//public class PixyThread extends Thread



	public class PixyThread implements Runnable{


	public void run(){

		while (!Thread.interrupted()) {
			try{

				testPixy.pixyTest();

				Thread.sleep(2000);


				} catch (Exception e) {
					//System.out.println(e);
				}

				}

				}		
			}//end of PixyThread


	public void BNO055_IMU () {
		
		boolean imuPresent;

		double [] imuVector;

		imuPresent = imu.isSensorPresent();

		if (imuPresent == true) {
			System.out.println("The imu is nominal");
		}
		else {
			System.out.println("The imu is not nominal and it's all kate's fault");
		}


		imuVector = imu.getVector();

		System.out.println("Vector: " + imuVector[0] );

		SmartDashboard.putNumber("BNO055",  imuVector[0]);

	}

	public void NavX () {

		SmartDashboard.putNumber("IMU_Yaw",  ahrs.getYaw());

		System.out.println("IMU_Yaw " + ahrs.getYaw());

	}//end of NavX test method


public class Count{
	
	int counterValue;

	Count(){
	
	}
		
	public void incrementAutoCounter(){

		counterValue++;
	}

	public void decrementAutoCounter(){

		counterValue--;
	}	

	public void resetCounter(){
		counterValue = 0;
	}

	public int returnCounterValue(){

		return counterValue;
	}

}//end Count class
	
	/*Autonomous Attempt
	 * 
	*/
	
	public void autonomousInit() { //This method is called once each time the robot enters autonomous mode
	     timer.reset(); // Resets the timer to 0
	     timer.start(); // Start counting

		encR.reset();
		encL.reset();


		int countR;
		int countL;

		double distanceR;
		double distanceL;
		
		autonomousCounter.resetCounter();
		

	}

	public void autonomousPeriodic() { //This method is called each time the robot recieves a packet instructing the robot to be in autonomous enabled mode
	     // Drive for 2 seconds
	

		_drive.arcadeDrive(0.4111234, 0.0);





		/*
if(autonomousCounter.returnCounterValue() <= 2){
		while(ahrs.getYaw() <= 90.0){

			_drive.arcadeDrive(0.0, 0.4);

		}

		NavX();

		BNO055_IMU();

		while(ahrs.getYaw() >= -90.0){

			_drive.arcadeDrive(0.0, -0.4);

		}

		_drive.arcadeDrive(0.0, 0.0);

		autonomousCounter.incrementAutoCounter();

		NavX();

		BNO055_IMU();

	}
else{
			_drive.arcadeDrive(0.0, 0.0);
		}
*/
/*
		if(autonomousCounter.returnCounterValue() <= 2){
			if(timer.get() <= 1.0){
				_drive.arcadeDrive(0.0, 0.0);
			}

			else if(0.0 <= ahrs.getYaw() && ahrs.getYaw() <= 90.0)
			 {
				 _drive.arcadeDrive(0.0, 0.4);
			 }//end of else 
			else if(0.0 > ahrs.getYaw() && ahrs.getYaw() > -90.0)
			 {
				 _drive.arcadeDrive(0.0, -0.4);
			 }//end of else 
			else if(90.0 > ahrs.getYaw() || ahrs.getYaw() <= 359.0)
			 {
				 _drive.arcadeDrive(0.0, -0.4);
			 }//end of else 

			else {
	    	 _drive.arcadeDrive(0.0, 0.0);  // stop robot
			 timer.reset();
			 autonomousCounter.incrementAutoCounter();
			
	    	}
			}//end of if statement

		else{
			_drive.arcadeDrive(0.0, 0.0);
		}
*/
		 /*
	     if (timer.get() < 5.0) {
	    	 _drive.arcadeDrive(0.4, 0.0); // drive forwards half speed
	     }else if(5.0 <= timer.get() && timer.get() < 9.0) {
	    	 _drive.arcadeDrive(0.0, 0.4);
	    	
	     } 
	     else {
	    	 _drive.arcadeDrive(0.0, 0.0);  // stop robot
	     }
	     */

		int countR;
		int countL;

		countR = encR.getRaw();
		countL = encL.getRaw();

		//message();


			/* EXPLANATION OF arcadeDrive function***

			_drive.arcadeDrive   (forward, turn)
			
				forward is the Y joystick input 
					POSITVE values = FORWARD  NEGATIVE values = REVERSE
				turn is the Z joystick input
					POSITIVE values = RIGHT   NEGATIVE values = LEFT
			
			 */
		
		/*

	    while((countR > -2000) || (countL < 2000)){

			_drive.arcadeDrive(0.4, 0.0); // drive forwards half speed

		
		countR = encR.getRaw();
		countL = encL.getRaw();


			message();

		}

		_drive.arcadeDrive(0.0, 0.0);  // stop robot

		*/
	
	}
	
	
	
	/*
	 * END of Autonomous
	 */
	public void teleopInit() { //This method is called once each time the robot enters autonomous mode
	
			PixyExample testPixy = new PixyExample();

		    //Thread ThreadPixy1 = new Thread(new PixyThread());

			timer.reset(); // Resets the timer to 0
	     	timer.start(); // Start counting

			System.out.println("teleopInit ran");//print the data just to see


	}



	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		/* sign this so forward is positive */
		//DigitalOutput = pulse(1.00);

		NavX();

		//BNO055_IMU();

		controlShifter();

		//message();
		
		//***move to robot init */
		//PixyExample testPixy = new PixyExample();

		//testPixy.pixyTest();

		


		//driveEncoders();
		
		double forward = -1.0 * _joy.getY();
		/* sign this so right is positive. */
		double turn = +1.0 * _joy.getZ();
		/* deadband */
		if (Math.abs(forward) < 0.10) {
			/* within 10% joystick, make it zero */
			forward = 0;
		}
		if (Math.abs(turn) < 0.10) {
			/* within 10% joystick, make it zero */
			turn = 0;
		}
		/* print the joystick values to sign them, comment
		 * out this line after checking the joystick directions. */
		System.out.println("JoyY:" + forward + "  turn:" + turn );
			
		
		//Runs Pixy Test Navigation
			
		if(forward == 0.0 && turn == 0.0){
			testPixy.pixyTest();
			}
		
		
/*
		if(forward == 0.0 && turn == 0.0){
			testPixy.mxpTest();
			}
*/
		/* drive the robot, when driving forward one side will be red.  
		 * This is because DifferentialDrive assumes 
		 * one side must be negative */
		_drive.arcadeDrive(forward, turn);
	}//end of teleopPeriodic


	public void testInit() { //This method is called once each time the robot enters test mode
	
		 timer.start(); // Start counting


	}
	
	
	public void testPeriodic() {
		

		

	     //timer.reset(); // Resets the timer to 0
	    


		LiveWindow.run();
	
	     // Drive for 2 seconds
		 
	     if (timer.get() < 5.0) {
	    	 _drive.arcadeDrive(0.4, 0.0); // drive forwards half speed
	     }else if(5.0 <= timer.get() && timer.get() < 9.0) {
	    	 _drive.arcadeDrive(0.0, 0.4);
	    
	     } 
	     else {
	    	 _drive.arcadeDrive(0.0, 0.0);  // stop robot
	     }
	    



	
	
	}//end of testPeriodic









}//end of IterativeRobot

	//CameraServer.getInstance().startAutomaticCapture();


		/*

				PixyThread ThreadPixy1 = new PixyThread();
		new Thread(ThreadPixy1).start();


			PixyThread ThreadPixy1 = new PixyThread();
			new Thread(ThreadPixy1).start();



		Thread ThreadPixy1 = new Thread(new PixyThread());

		ThreadPixy1.start();
		
		ThreadPixy1.interrupted();

		
		ThreadPixy1 = null;
		System.gc();
		






		new Thread(() -> {
                UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
                camera.setResolution(640, 480);
                
                CvSink cvSink = CameraServer.getInstance().getVideo();
                CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
                
                Mat source = new Mat();
                Mat output = new Mat();
                
                while(!Thread.interrupted()) {
                    cvSink.grabFrame(source);
                    Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                    outputStream.putFrame(output);
                }
            }).start();
		
		
		
		
//Integer.parseInt(packet);


//Integer xPosition =Integer.valueOf(packet);;


//xPosition = Double.parseDouble(packet);

//System.out.println("XPos: " + xPosition);// pkt.xprint the data just to see
		
		
		
		
		
		
		
		
		
		
		
		*/

