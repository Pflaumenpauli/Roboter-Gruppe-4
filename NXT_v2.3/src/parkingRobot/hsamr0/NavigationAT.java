package parkingRobot.hsamr0;

import lejos.geom.Line;
import lejos.geom.Point;
import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.robotics.navigation.Pose;

import parkingRobot.INavigation;
import parkingRobot.IPerception;
//import parkingRobot.INavigation.ParkingSlot;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.IMonitor;

import parkingRobot.hsamr0.NavigationThread;


/**
 * A executable basic example implementation of the corresponding interface provided by the Institute of Automation with
 * limited functionality:
 * <p>
 * In this example only the both encoder sensors from the {@link IPerception} implementation are used for periodically calculating
 * the robots position and corresponding heading angle (together called 'pose'). Neither any use of the map information or other
 * perception sensor information is used nor any parking slot detection is performed, although the necessary members are already
 * prepared. Advanced navigation calculation and parking slot detection should be developed and invented by the students.  
 * 
 * @author IfA
 */
public class NavigationAT implements INavigation{
	
	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorLeft	=	0;
	
	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft							=	null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderRight							=	null;
	
	/**
	 * reference to data class for measurement of the left wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft 	= 	null;
	/**
	 * reference to data class for measurement of the right wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight	= 	null;
	
	/**
	 * reference to {@link IPerception.OdoSensor} class for mouse odometry sensor to measure the ground displacement
	 * between actual an last request
	 */
	IPerception.OdoSensor mouseodo = null;	
		
	/**
	 * reference to data class for measurement of the mouse odometry sensor to measure the ground displacement between
	 * actual an last request and the corresponding time difference
	 */
	IPerception.OdoDifferenceMeasurement mouseOdoMeasurement = null;
	
	/**
	 * distance from optical sensor pointing in driving direction to obstacle in mm
	 */
	double frontSensorDistance		=	0;
	/**
	 * distance from optical sensor pointing to the right side of robot to obstacle in mm (sensor mounted at the front)
	 */
	double frontSideSensorDistance	=	0;
	/**
	 * distance from optical sensor pointing in opposite of driving direction to obstacle in mm
	 */
	double backSensorDistance		=	0;
	/**
	 * distance from optical sensor pointing to the right side of robot to obstacle in mm (sensor mounted at the back)
	 */
	double backSideSensorDistance	=	0;
	
	/**
	 * robot specific constant: radius of left wheel
	 */
	static final double LEFT_WHEEL_RADIUS	= 	0.02810; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: radius of right wheel
	 */
	static final double RIGHT_WHEEL_RADIUS	= 	0.02765; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: distance between wheels
	 */
	static final double WHEEL_DISTANCE		= 	0.1511; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: usual distance between trail and wall
	 */
	static final double TRSH_SG = 40;	
	/**
	 * robot specific constant: length of robot
	 */
	static final double LGNT_ROBOT = 0.45;
	
	// Pose verification
	int 	Po_CORNER_ID = 0;
	double 	Po_ExpAng = 0;
	
	short 	Po_Corn		= 0;
	short 	Po_AxeP 	= 0;
	short	Po_RoundF	= 0;
	
	static final double TRSH_W = 60;
	static final double TRSH_G = 90;
	
	// Parking
	double[] Pk_DIST_FS = new double [4];
	double[] Pk_DIST_BS = new double [4];
	
	short Pk_burstFS = 0;
	short Pk_burstRS = 0;
	short Pk_burstFE = 1;
	short Pk_burstRE = 1;
	
	Point Pk_PosF1 = new Point(0,0);
	Point Pk_PosF2 = new Point(0,0);
	Point Pk_PosR1 = new Point(0,0);
	Point Pk_PosR2 = new Point(0,0);
	
	int Pk_counter = 0;
	int Pk_update  = 0;
	
	int Pk_new = 0;
	
	/**
	 * 
	 */
	INavigation.ParkingSlot[] Pk_slotList = new ParkingSlot[10];
	
	/**
	 * map array of line references, whose corresponding lines form a closed chain and represent the map of the robot course
	 */
	Line[] map 								= null;
	/**
	 * reference to the corresponding main module Perception class
	 */
	IPerception perception 	        		= null;
	/**
	 * reference to the corresponding main module Monitor class
	 */
	IMonitor monitor = null;
	/**
	 * indicates if parking slot detection should be switched on (true) or off (false)
	 */
	boolean parkingSlotDetectionIsOn		= false;
	/**
	 * pose class containing bundled current X and Y location and corresponding heading angle phi
	 */
	Pose pose								= new Pose();

	/**
	 * thread started by the 'Navigation' class for background calculating
	 */
	NavigationThread navThread = new NavigationThread(this);

	/**
	 * provides the reference transfer so that the class knows its corresponding perception object (to obtain the measurement
	 * information from) and starts the navigation thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param monitor corresponding main module Monitor class object
	 */
	public NavigationAT(IPerception perception, IMonitor monitor){
		this.perception   = perception;
		this.monitor = monitor;
		this.encoderLeft  = perception.getNavigationLeftEncoder();
		this.encoderRight = perception.getNavigationRightEncoder();
		this.mouseodo = perception.getNavigationOdo();		
		
		navThread.setPriority(Thread.MAX_PRIORITY - 1);
		navThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		navThread.start();
	}
	
	// Inputs
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#setMap(lejos.geom.Line[])
	 */
	public void setMap(Line[] map){
		this.map = map;
	}
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#setDetectionState(boolean)
	 */
	public void setDetectionState(boolean isOn){
		this.parkingSlotDetectionIsOn = isOn;
	}
	
	// 	Class control
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#updateNavigation()
	 */
	public synchronized void updateNavigation(){	
		this.updateSensors();
		this.calculateLocation();
		if (this.parkingSlotDetectionIsOn)
				this.detectParkingSlot();
		
		// MONITOR (example)
//		monitor.writeNavigationComment("Navigation");
	}
	
	// Outputs
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#getPose()
	 */
	public synchronized Pose getPose(){
		return this.pose;
	}
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#getParkingSlots()
	 */
	public synchronized ParkingSlot[] getParkingSlots() 
	{
		int i = 0, j = 0;
		
		for (i = 0; Pk_slotList[i] != null; i++);
		
		INavigation.ParkingSlot[] listOut = new ParkingSlot[i];
		
		for (j = 0; j <= i; j++)
		{
			listOut[j] = Pk_slotList[j];
		}
		
		if (i != 0)
		{
			return listOut;
		}
		else
		{
			return null;
		}
	}
	
	// Private methods
	/**
	 * calls the perception methods for obtaining actual measurement data and writes the data into members
	 */
	private void updateSensors()
	{		
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		
		this.angleMeasurementLeft  	= this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight 	= this.encoderRight.getEncoderMeasurement();

		this.mouseOdoMeasurement	= this.mouseodo.getOdoMeasurement();

		this.frontSensorDistance	= perception.getFrontSensorDistance();
		this.frontSideSensorDistance = perception.getFrontSideSensorDistance();
		this.backSensorDistance		= perception.getBackSensorDistance();
		this.backSideSensorDistance	= perception.getBackSideSensorDistance();
	}		 	
	
	/**
	 * calculates the axe of the movement
	 */
	private short getHeadingAxe()
	{
		// Condition based on the angle change
		double difA = 0;
		short  movDir = 0;

		// Difference between the real and ideal angle
		difA = Math.abs((this.pose.getHeading()/Math.PI*180) - Po_ExpAng);
		
		// Axe detection
		if ((Po_CORNER_ID % 2) == 0)
		{
			// Movement in x
			if (((Po_CORNER_ID == 0) && ((difA > 70) && (100*this.pose.getX() > 165))) || ((Po_CORNER_ID == 2) && ((difA > 70) && (100*this.pose.getX() < 160))) || ((Po_CORNER_ID == 4) && ((difA > 70) && (100*this.pose.getX() < 35))) || ((Po_CORNER_ID == 6) && ((difA > 70) && (100*this.pose.getX() < 5))))
			//if (difA > 70)
			{
				movDir = 1;			// y direction
				Sound.beepSequenceUp();
			}
			else
			{
				movDir = 0;			// x direction
			}
		}
		else
		{
			// Movement in y
			if (((Po_CORNER_ID == 1) && ((difA > 30) && (100*this.pose.getY() > 55))) || ((Po_CORNER_ID == 3) && ((100*this.pose.getY() < 35))) || ((Po_CORNER_ID == 5) && ((difA > 70) && (100*this.pose.getY() > 55))) || ((Po_CORNER_ID == 7) && (difA > 70) && (100*this.pose.getY() < 5)))
			//if (difA > 70)
			{
				movDir = 0;			// x direction
				Sound.beepSequence();
			}
			else
			{
				movDir = 1;			// y direction
			}
		}
		
		if (((Po_AxeP == 0) && (movDir == 1)) || ((Po_AxeP == 1) && (movDir == 0)))
		{
			if (Po_CORNER_ID < 7)
			{
				Po_CORNER_ID = Po_CORNER_ID + 1;
			}
			else
			{
				Po_CORNER_ID = 0;
				
				if (this.parkingSlotDetectionIsOn)
				{
					Po_RoundF = 1;		// Round finished, don't add any new parking slots
				}
			}

			Po_Corn = 1;
		}
		else
		{
			Po_Corn = 0;
		}
		
		Po_AxeP = movDir;
		
		//LCD.drawString("Corn_ID: " + (Po_CORNER_ID), 0, 6);
		
		return movDir;
	}
	
	/**
	 * calculates the robot pose from the measurements
	 */
	private void calculateLocation()
	{
		double leftAngleSpeed 	= this.angleMeasurementLeft.getAngleSum()  / ((double)this.angleMeasurementLeft.getDeltaT()/1000);  //degree/seconds
		double rightAngleSpeed 	= this.angleMeasurementRight.getAngleSum() / ((double)this.angleMeasurementRight.getDeltaT()/1000); //degree/seconds

		double vLeft	= (leftAngleSpeed  * Math.PI * LEFT_WHEEL_RADIUS ) / 180 ; //velocity of left  wheel in m/s
		double vRight	= (rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180 ; //velocity of right wheel in m/s		
		double w 		= (vRight - vLeft) / WHEEL_DISTANCE; //angular velocity of robot in rad/s
		
		Double R 	= new Double(( WHEEL_DISTANCE / 2 ) * ( (vLeft + vRight) / (vRight - vLeft) ));								
		
		double ICCx = 0;
		double ICCy = 0;

		double W_xResult 		= 0;
		double W_yResult 		= 0;
		double W_angleResult 	= 0;
		
		double E_xResult 		= 0;
		double E_yResult 		= 0;
		double E_angleResult 	= 0;
		
		//short axe = 0;
		
		double deltaT       = ((double)this.angleMeasurementLeft.getDeltaT())/1000;
		
		// Odometry calculations
		if (R.isNaN()) 				//robot don't move
		{
			W_xResult		= this.pose.getX();
			W_yResult		= this.pose.getY();
			W_angleResult 	= this.pose.getHeading();
		} 
		else if (R.isInfinite()) 	//robot moves straight forward/backward, vLeft==vRight
		{ 
			W_xResult		= this.pose.getX() + vLeft * Math.cos(this.pose.getHeading()) * deltaT;
			W_yResult		= this.pose.getY() + vLeft * Math.sin(this.pose.getHeading()) * deltaT;
			W_angleResult 	= this.pose.getHeading();
		} 
		else 
		{			
			ICCx = this.pose.getX() - R.doubleValue() * Math.sin(this.pose.getHeading());
			ICCy = this.pose.getY() + R.doubleValue() * Math.cos(this.pose.getHeading());
		
			W_xResult 		= Math.cos(w * deltaT) * (this.pose.getX()-ICCx) - Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
			W_yResult 		= Math.sin(w * deltaT) * (this.pose.getX()-ICCx) + Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
			W_angleResult 	= this.pose.getHeading() + w * deltaT;
		}
		
		this.pose.setLocation((float)W_xResult, (float)W_yResult);
		this.pose.setHeading((float)W_angleResult);
		
		// Conversion to grads
		W_angleResult = W_angleResult/Math.PI*180;
		
		// Get the heading axe
		//axe = getHeadingAxe();
		getHeadingAxe();
		
		// Verify the coordinates and the heading angle
		if (Po_Corn == 1)
		{
			switch (Po_CORNER_ID)
			{
				case 0:
					E_xResult = 0;
					E_yResult = 0;
					Po_ExpAng = 0;
					break;
				case 1:
					E_xResult = 1.8;
					E_yResult = 0.11;
					Po_ExpAng = 90;
					break;
				case 2:
					E_xResult = 1.6;
					E_yResult = 0.6;
					Po_ExpAng = 180;
					break;
				case 3:
					E_xResult = 1.5;
					E_yResult = 0.5;
					Po_ExpAng = 270;
					break;
				case 4:
					E_xResult = 1.5;
					E_yResult = 0.3;
					Po_ExpAng = 180;
					break;
				case 5:
					E_xResult = 0.3;
					E_yResult = 0.3;
					Po_ExpAng = 90;
					break;
				case 6:
					E_xResult = 0.3;
					E_yResult = 0.6;
					Po_ExpAng = 180;
					break;
				case 7:
					E_xResult = 0;
					E_yResult = 0.6;
					Po_ExpAng = 270;
					break;	
			}
			
			E_angleResult = W_angleResult;
			Po_Corn = 0;
		}
		else
		{
			int block = 0;
			// white = 0, black = 2, grey = 1
			if ((lineSensorLeft == 0) && (lineSensorRight == 0) && (block == 0)) 	// Robot moves on the black line
			{
				switch (Po_CORNER_ID)								// Even numbers - x, Odd numbers - y
				{
				case 0:  
					if (this.pose.getX() < 1.6)
					{
						E_angleResult = Po_ExpAng;
						E_yResult = 0;
					}
					else
					{
						E_angleResult = W_angleResult;
						E_yResult = W_yResult;
					}
					E_xResult = W_xResult;
					break;
					
				case 1:  
					if (this.pose.getY() < 0.4)
					{
						E_angleResult = Po_ExpAng;
						E_xResult = 1.8;
					}
					else
					{
						E_angleResult = W_angleResult; 
						E_xResult = W_xResult;
					}
					E_yResult = W_yResult;
					break;
					
				case 2:
					if (this.pose.getX() > 1.65)
					{
						E_angleResult = Po_ExpAng;
						E_yResult = 0.6;
					}
					else
					{
						E_angleResult = W_angleResult;
						E_yResult = W_yResult;
					}
					E_xResult = W_xResult;
					break;
					
				case 3:
					if (this.pose.getY() > 0.4)
					{
						E_angleResult = Po_ExpAng;
						E_xResult = 1.5;
					}
					else
					{
						E_angleResult = W_angleResult; 
						E_xResult = W_xResult;
					}
					E_yResult = W_yResult;
					break;
					
				case 4:  
					if (this.pose.getX() > 0.4)
					{
						E_angleResult = Po_ExpAng;
						E_yResult = 0.3;
					}
					else
					{
						E_angleResult = W_angleResult;
						E_yResult = W_yResult;
					}
					E_xResult = W_xResult;
					break;
					
				case 5:
					if (this.pose.getY() < 0.5)
					{
						E_angleResult = Po_ExpAng;
						E_xResult = 0.3;
					}
					else
					{
						E_angleResult = W_angleResult; 
						E_xResult = W_xResult;
					}
					E_yResult = W_yResult;
					break;
					
				case 6:  
					if (this.pose.getX() > 0.1)
					{
						E_angleResult = Po_ExpAng;
						E_yResult = 0.6;
					}
					else
					{
						E_angleResult = W_angleResult;
						E_yResult = W_yResult;
					}
					E_xResult = W_xResult;
					break;
					
				case 7:
					if (this.pose.getY() > 0.1)
					{
						E_angleResult = Po_ExpAng;
						E_xResult = 0;
					}
					else
					{
						E_angleResult = W_angleResult; 
						E_xResult = W_xResult;
					}
					E_yResult = W_yResult;
					break;
					
				default: 
					E_angleResult = W_angleResult;
					E_yResult = W_yResult;
					E_xResult = W_xResult;
					break;
				}
			}
			else
			{
				E_xResult = W_xResult;
				E_yResult = W_yResult;
				E_angleResult = W_angleResult;
			}
		}
		
		//LCD.drawString("AngRs: " + (E_angleResult), 0, 7);
		
		// Conversion to rads
		W_angleResult = W_angleResult*Math.PI/180;
		E_angleResult = E_angleResult*Math.PI/180;
		
		this.pose.setLocation((float)E_xResult, (float)E_yResult);
		this.pose.setHeading((float)E_angleResult);
		
		/*
		// Integration deviation correction
		this.pose.setLocation((float)(W_xResult), (float)(W_yResult + (0.01*22.4)/175));
		this.pose.setHeading((float)((W_angleResult + 14/175)*Math.PI/180));
		*/
	}

	/**
	 * detects parking slots and manage them by initializing new slots, re-characterizing old slots or merge old and detected slots. 
	 */
	private void detectParkingSlot()
	{		
		Point PosS = new Point(0,0);
		Point PosE = new Point(0,0);
		
		double sum_F = 0;
		double sum_B = 0;
		
		double distance_F = 0;
		double distance_B = 0;
		
		int SlotID = Pk_counter;
		INavigation.ParkingSlot.ParkingSlotStatus SlotStatus = ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING;
		
		short axe = getHeadingAxe();
		
		for (int i = 1; i <= 3; i++)
		{
			Pk_DIST_FS[i] = Pk_DIST_FS[i-1];
			sum_F = Pk_DIST_FS[i] + sum_F;
			
			Pk_DIST_BS[i] = Pk_DIST_BS[i-1];
			sum_B = Pk_DIST_BS[i] + sum_B;
		}
		
		Pk_DIST_FS[0] = frontSensorDistance;
		distance_F = (sum_F + Pk_DIST_FS[0])/4;
		//distance_F = frontSensorDistance;
		
		Pk_DIST_BS[0] = backSideSensorDistance;
		distance_B = (sum_B + Pk_DIST_BS[0])/4;
		//distance_B = backSideSensorDistance;
		
		LCD.drawString("Dist_F: " + (distance_F), 0, 6);
		LCD.drawString("Dist_B: " + (distance_B), 0, 7);
		
		// Saving the begin point of the PS
		if ((Pk_DIST_FS[0] >= TRSH_SG) && (distance_F >= TRSH_SG) && (Pk_burstFS == 0) && (((100*this.pose.getX() > 0) && (100*this.pose.getX() < 180)) || ((100*this.pose.getX() > 135) && (100*this.pose.getX() < 45))|| ((100*this.pose.getY() > 10) && (100*this.pose.getY() < 50))))
		{
			Pk_PosF1.setLocation(this.pose.getX(), this.pose.getY());
			Pk_burstFS = 1;
			Pk_burstFE = 0;
			//Sound.beep();
		}
		
		if ((Pk_DIST_BS[0] >= TRSH_SG) && (distance_B >= TRSH_SG) && (Pk_burstRS == 0) && (((100*this.pose.getX() > 0) && (100*this.pose.getX() < 180)) || ((100*this.pose.getX() > 135) && (100*this.pose.getX() < 45))|| ((100*this.pose.getY() > 10) && (100*this.pose.getY() < 50))))
		{
			Pk_PosR1.setLocation(this.pose.getX(), this.pose.getY());
			Pk_burstRS = 1;
			Pk_burstRE = 0;
			//Sound.twoBeeps();
		}
				
		// Saving the end point of the PS
		if ((Pk_DIST_FS[0] <= TRSH_SG) && (distance_F <= TRSH_SG) && (Pk_burstFE == 0) && (((100*this.pose.getX() > 0) && (100*this.pose.getX() < 180)) || ((100*this.pose.getX() > 135) && (100*this.pose.getX() < 45))|| ((100*this.pose.getY() > 10) && (100*this.pose.getY() < 50))))
		{
			Pk_PosF2.setLocation(this.pose.getX(), this.pose.getY());
			Pk_burstFS = 0;
			Pk_burstFE = 1;
			//Sound.beep();
		}
		
		if ((Pk_DIST_BS[0] <= TRSH_SG) && (distance_B <= TRSH_SG) && (Pk_burstRE == 0) && (((100*this.pose.getX() > 0) && (100*this.pose.getX() < 180)) || ((100*this.pose.getX() > 135) && (100*this.pose.getX() < 45))|| ((100*this.pose.getY() > 10) && (100*this.pose.getY() < 50))))
		{
			Pk_PosR2.setLocation(this.pose.getX(), this.pose.getY());
			Pk_burstRS = 0;
			//burstRE = 1;
			//Sound.twoBeeps();
		}
		
		if (Po_RoundF == 0)			// Saving new parking slots
		{	
			if ((Pk_burstRS == 0) && (Pk_burstRE == 0) && (Pk_counter < 10))
			{
				PosS.setLocation(((Pk_PosF1.getX() + Pk_PosR1.getX())/2), ((Pk_PosF1.getY() + Pk_PosR1.getY())/2));
				PosE.setLocation(((Pk_PosF2.getX() + Pk_PosR2.getX())/2), ((Pk_PosF2.getY() + Pk_PosR2.getY())/2));
				
				// Evaluation of the slot
				if (axe == 0)
				{
					if ((PosE.getX() - PosS.getX()) > LGNT_ROBOT)
					{
						SlotStatus = ParkingSlotStatus.SUITABLE_FOR_PARKING;
					}
					else
					{
						SlotStatus = ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING;
					}
				}
				else
				{
					if ((PosE.getY() - PosS.getY()) > LGNT_ROBOT)
					{
						SlotStatus = ParkingSlotStatus.SUITABLE_FOR_PARKING;
					}
					else
					{
						SlotStatus = ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING;
					}
				}
				
				Pk_slotList[Pk_counter] = new INavigation.ParkingSlot(SlotID, PosE, PosS, SlotStatus, 0);
				Pk_counter ++;
				
				Pk_burstRE = 1;
				Pk_new = 1;
				
				Sound.beepSequence();
			}
		}
		else					// Updating the old slots
		{
			if ((Pk_burstRS == 0) && (Pk_burstRE == 0) && (Pk_update <= Pk_counter))
			{
				PosS.setLocation(((Pk_PosF1.getX() + Pk_PosR1.getX())/2), ((Pk_PosF1.getY() + Pk_PosR1.getY())/2));
				PosE.setLocation(((Pk_PosF2.getX() + Pk_PosR2.getX())/2), ((Pk_PosF2.getY() + Pk_PosR2.getY())/2));
				
				// Evaluation of the slot
				if (axe == 0)
				{
					if ((PosE.getX() - PosS.getX()) > LGNT_ROBOT)
					{
						SlotStatus = ParkingSlotStatus.SUITABLE_FOR_PARKING;
					}
					else
					{
						SlotStatus = ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING;
					}
				}
				else
				{
					if ((PosE.getY() - PosS.getY()) > LGNT_ROBOT)
					{
						SlotStatus = ParkingSlotStatus.SUITABLE_FOR_PARKING;
					}
					else
					{
						SlotStatus = ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING;
					}
				}
				
				Pk_slotList[Pk_update] = new INavigation.ParkingSlot(SlotID, PosE, PosS, SlotStatus, 0);
				
				if (Pk_update < Pk_counter)
				{
					Pk_update ++;
				}
				else
				{
					Pk_update = 0;
				}
				
				Pk_burstRE = 1;
			}
		}
		
		return; // data are saved in the shared variable
	}
}