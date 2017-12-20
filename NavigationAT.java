package parkingRobot.hsamr0;

import lejos.geom.Line;
import lejos.geom.Point;
import lejos.robotics.navigation.Pose;

import parkingRobot.INavigation;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;


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
	static final double LEFT_WHEEL_RADIUS	= 	0.0281; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: radius of right wheel
	 */
	static final double RIGHT_WHEEL_RADIUS	= 	0.02765; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: distance between wheels
	 */
	static final double WHEEL_DISTANCE		= 	0.1511; // only rough guess, to be measured exactly and maybe refined by experiments

	/**
	 * Added constants and variables
	 */
	// Radodometrie
	double W_xResult 	= 0;
	double W_yResult 	= 0;
	double W_aResult	= 0;		
	double W_xResultP	= 0;
	double W_yResultP	= 0;
	double W_dist		= 0;
	
	// Mauseodometrie
	double M_xResult 	= 0;
	double M_yResult 	= 0;
	double M_aResult	= 0;	
	double M_deltaX		= 0;
	double M_deltaY		= 0;
	double M_dist		= 0;
	
	double M_xPosOld	= 0;
	double M_yPosOld	= 0;
	
	// Axe detection
	double[] DELTA_X = new double [10];
	double[] DELTA_Y = new double [10];
	
	short AxeP = 0;
	short corn = 0;
	
	// Corner detection
	double[] DIST_F = {0,0,0,0,0};
	double[] DIST_B = {0,0,0,0,0};
	int CORNER_ID = 1;
	
	static final double TRSH_DISTN = 370;
	static final double TRSH_DIST4 = 0;
	static final double TRSH_DIST5 = 0;
	
	// Angle verification
	static final double TRSH_B = 30;
	static final double TRSH_W = 60;
	static final double TRSH_G = 90;
	
	// Parking
	double[] DIST_FS = {0,0,0,0,0};
	double[] DIST_BS = {0,0,0,0,0};
	
	short burstFS = 0;
	short burstRS = 0;
	short burstFE = 1;
	short burstRE = 1;
	
	Point PosF1 = new Point(0,0);
	Point PosF2 = new Point(0,0);
	Point PosR1 = new Point(0,0);
	Point PosR2 = new Point(0,0);
	
	int counter = 0;
	public INavigation.ParkingSlot[] slotList = new ParkingSlot[10];
	
	static final double TRSH_SG = 0.13;	
	static final double LGNT_ROBOT = 0.45;
	
	
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
	public synchronized void updateNavigation()
	{	
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
		return null;
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

		this.frontSensorDistance	 = perception.getFrontSensorDistance();
		this.frontSideSensorDistance = perception.getFrontSideSensorDistance();
		this.backSensorDistance		 = perception.getBackSensorDistance();
		this.backSideSensorDistance  = perception.getBackSideSensorDistance();
	}
	
	/**
	 * calculates the wheel odometry
	 */
	private void radOdometrie()
	{
		double leftAngleSpeed 	= this.angleMeasurementLeft.getAngleSum()  / ((double)this.angleMeasurementLeft.getDeltaT()/1000);  //degree/seconds
		double rightAngleSpeed 	= this.angleMeasurementRight.getAngleSum() / ((double)this.angleMeasurementRight.getDeltaT()/1000); //degree/seconds

		double vLeft	= (leftAngleSpeed  * Math.PI * LEFT_WHEEL_RADIUS ) / 180 ; //velocity of left  wheel in m/s
		double vRight	= (rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180 ; //velocity of right wheel in m/s		
		double w 		= (vRight - vLeft) / WHEEL_DISTANCE; //angular velocity of robot in rad/s							+++++
		
		Double R 		= new Double(( WHEEL_DISTANCE / 2 ) * ( (vLeft + vRight) / (vRight - vLeft) ));
		double deltaT   = ((double)this.angleMeasurementLeft.getDeltaT())/1000;
		
		double ICCx 	= 0;
		double ICCy 	= 0;
		
		if (R.isNaN()) 			//robot don't move
		{ 
			W_xResult		= this.pose.getX();
			W_yResult		= this.pose.getY();
			W_aResult 		= this.pose.getHeading();
		} 
		else if (R.isInfinite()) //robot moves straight forward/backward, vLeft==vRight
		{ 
			W_xResult		= this.pose.getX() + vLeft * Math.cos(this.pose.getHeading()) * deltaT;
			W_yResult		= this.pose.getY() + vLeft * Math.sin(this.pose.getHeading()) * deltaT;
			W_aResult		= this.pose.getHeading();
		} 
		else 
		{			
			ICCx = this.pose.getX() - R.doubleValue() * Math.sin(this.pose.getHeading());
			ICCy = this.pose.getY() + R.doubleValue() * Math.cos(this.pose.getHeading());
		
			W_xResult 		= Math.cos(w * deltaT) * (this.pose.getX()-ICCx) - Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
			W_yResult 		= Math.sin(w * deltaT) * (this.pose.getX()-ICCx) + Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
			W_aResult 		= this.pose.getHeading() + w * deltaT;
		}
		/*
		W_deltaX = Math.abs(W_xResult - this.pose.getX());
		W_deltaY = Math.abs(W_yResult - this.pose.getY());
		
		W_dist	 = Math.sqrt(Math.pow(W_deltaX, 2) + Math.pow(W_deltaY, 2));	// Driven distance - wheel odometry
		W_CaRes  = Math.atan(W_dist/W_deltaX);						// Counted heading angle - wheel odometry ----- mozna neni potreba
		*/
	}
	
	/**
	 *  calculates the mouse odometry
	 */
	private void mauseOdometrie()
	{
		short axe = 0;
		
		axe = getHeadingAxe();
		/*	
		switch (this.CORNER_ID)
		{
		case 0:
		case 2:
		case 4:
		case 6:
			this.M_deltaX = this.mouseOdoMeasurement.getUSum()/1000;
			this.M_deltaY = this.mouseOdoMeasurement.getVSum()/1000;
			break;
		case 1:
		case 3:
		case 5:
		case 7:
			this.M_deltaX = this.mouseOdoMeasurement.getVSum()/1000;
			this.M_deltaY = this.mouseOdoMeasurement.getUSum()/1000;
			break;
		}
		*/
		
		switch (axe)
		{
		case 0:
			M_deltaX = this.mouseOdoMeasurement.getUSum();
			M_deltaY = this.mouseOdoMeasurement.getVSum();
			break;
		case 1:
			M_deltaX = this.mouseOdoMeasurement.getVSum();
			M_deltaY = this.mouseOdoMeasurement.getUSum();
			break;
		case 2:
			// Pripad s nedefinovanou osou pohybu
			M_deltaX = this.mouseOdoMeasurement.getUSum();
			M_deltaY = this.mouseOdoMeasurement.getVSum();
			break;
		}
		
		M_xResult = M_deltaX + M_xPosOld;
		M_yResult = M_deltaY + M_yPosOld;
		
		M_xPosOld = M_xResult;
		M_yPosOld = M_yResult;
		
		M_dist	 = Math.sqrt(Math.pow(M_deltaX, 2) + Math.pow(M_deltaY, 2));	// Driven distance - mouse sensor
		M_aResult  = Math.atan(M_deltaY/M_deltaX);								// Counted heading angle - mouse sensor
	}
	
	/**
	 * calculates the axe of the movement
	 */
	private short getHeadingAxe()
	{
		// On the base of wheel odometry
		double avgX = 0;
		double avgY = 0;
		
		double sumX = 0;
		double sumY = 0;
		
		short movDir = 0;

		for (int i = 1; i <= 9; i++)
		{
			DELTA_X[i] = DELTA_X[i-1];
			sumX = DELTA_X[i] + sumX;
			
			DELTA_Y[i] = DELTA_Y[i-1];
			sumY = DELTA_Y[i] + sumY;
		}
		
		//this.DELTA_X[0] = this.pose.getX();
		DELTA_X[0] = Math.abs(W_xResult - W_xResultP);
		sumX = sumX + DELTA_X[0];
		avgX = (sumX)/10;
		
		//this.DELTA_Y[0] = this.pose.getY();
		DELTA_Y[0] = Math.abs(W_yResult - W_yResultP);
		sumY = sumY + DELTA_Y[0];
		avgY = (sumY)/10;
		
		W_xResultP = W_xResult;
		W_yResultP = W_yResult;
		
		//if ((10*avgX) >= avgY)
		if ((10*sumX) >= sumY)
		{
			movDir = 0;			// x direction
		}
		//else if (avgX < (10*avgY))
		else if (sumX < (10*sumY))
		{
			movDir = 1;			// y direction
		}
		
		// mod 8
		if (((AxeP == 0) && (movDir == 1)) || ((AxeP == 1) && (movDir == 0)))
		{
			if (CORNER_ID < 7)
			{
				CORNER_ID ++;
			}
			else
			{
				CORNER_ID = 0;
			}
			
			corn = 1;
		}
		else
		{
			corn = 0;
		}
		
		AxeP = movDir;
		
		return movDir;
	}
	
	/**
	 * detects the corners 
	 */
	private short getCorner()
	{	
		double distance_F = 0;
		double distance_B = 0;
		double sum_F 	= 0;
		double sum_B 	= 0;
		
		short burst		= 0;
		short corner	= 0;
		
		for (int i = 1; i <= 4; i++)
		{
			DIST_F[i] = DIST_F[i-1];
			sum_F = DIST_F[i] + sum_F;
			
			DIST_B[i] = DIST_B[i-1];
			sum_B = DIST_B[i] + sum_B;
		}
		
		DIST_F[0] = frontSensorDistance;
		distance_F = (sum_F + DIST_F[0])/5;
		
		DIST_B[0] = backSensorDistance;
		distance_B = (sum_B + DIST_B[0])/5;
		
		if (CORNER_ID == 4)
		{
			if (distance_B <= TRSH_DIST4)
			{
				corner = 1;
				burst = 1;
			}
			else
			{
				corner = 0;
			}
		}
		else if (CORNER_ID == 5)
		{
			if (distance_F <= TRSH_DIST5)
			{
				corner = 1;
				burst = 1;
			}
			else
			{
				corner = 0;
			}
		}
		else
		{
			if (distance_F <= TRSH_DISTN)
			{
				corner = 1;
				burst = 1;
			}
			else
			{
				corner = 0;
			}
		}
		
		if (burst == 1)			// Incrementing the corner
		{
			burst = 0;
			
			if (CORNER_ID < 7)
			{
				CORNER_ID++;
			}
			else
			{
				CORNER_ID = 0;
			}
		}
		
		return corner;
	}
	
	/**
	 * calculates the robot pose from the measurements
	 */
	private void calculateLocation()
	{				
		double xResult 	= 0;
		double yResult 	= 0;
		double aResult	= 0;
		
		double xDiff_W = 0;
		double yDiff_W = 0;
		
		double xDiff_M = 0;
		double yDiff_M = 0;
		
		double xId = 0;
		double yId = 0;
		
		short[] aVerif = {0,0};
		short axe = 0;
		//short corn = 0;
		
		// Wheel odometry
		radOdometrie();
		
		// Detecting the axe of the movement
		axe = getHeadingAxe();
		
		// Detecting the corners
		//corn = getCorner();
		
		// Mouse sensor
		mauseOdometrie();
		
		// Data evaluation
			
			// Verify the coordinates
			if (corn == 1)
			{
				switch (this.CORNER_ID)
				{
					case 0:
						xResult = 0;
						yResult = 0;
						
						M_xPosOld = 0;
						M_yPosOld = 0;
						break;
					case 1:
						xResult = 1.8;
						yResult = 0;
						break;
					case 2:
						xResult = 1.8;
						yResult = 0.6;
						break;
					case 3:
						xResult = 1.5;
						yResult = 0.6;
						break;
					case 6:
						xResult = 0.3;
						yResult = 0.6;
						break;
					case 7:
						xResult = 0;
						yResult = 0.6;
						break;	
				}
			}
			else
			{
				if ((lineSensorLeft == 0) && (lineSensorRight == 0))
				{
					if (axe == 0)		// movement in x direction
					{
						//xResult = M_xResult;
						xResult = W_xResult;
						
						switch (CORNER_ID)
						{
						case 0: yId = 0; break;
						case 2:
						case 6: yId = 0.6; break;
						case 4: yId = 0.3; break;
						}
						
						yResult = yId;
					}
					else if (axe == 1)	// movement in y direction
					{
						//yResult = M_yResult;
						yResult = W_yResult;
						
						switch (CORNER_ID)
						{
						case 1: xId = 1.8; break;
						case 3: xId = 1.5; break;
						case 5: xId = 0.3; break;
						case 7: xId = 0; break;
						}

						xResult = xId;
					}
				}
				else if(((lineSensorLeft == 0) && (lineSensorRight == 2)) || ((lineSensorLeft == 2) && (lineSensorRight == 0)))
				{
					if (axe == 0)		// movement in x direction
					{
						xResult = W_xResult;
						
						switch (CORNER_ID)
						{
						case 0: yId = 0; break;
						case 2:
						case 6: yId = 0.6; break;
						case 4: yId = 0.3; break;
						}
						
						yDiff_W = Math.abs(W_yResult - yId);
						yDiff_M = Math.abs(M_yResult - yId);
						
						if (yDiff_M <= yDiff_W)
						{
							yResult = M_yResult;
						}
						else
						{
							yResult = W_yResult;
						}
						
						// Test
						yResult = W_yResult;
					}
					else if (axe == 1)	// movement in y direction
					{
						yResult = W_yResult;
						
						switch (CORNER_ID)
						{
						case 1: xId = 1.8; break;
						case 3: xId = 1.5; break;
						case 5: xId = 0.3; break;
						case 7: xId = 0; break;
						}
						
						xDiff_W = Math.abs(W_xResult - xId);
						xDiff_M = Math.abs(M_xResult - xId);
						
						if (xDiff_M <= xDiff_W)
						{
							xResult = M_xResult;
						}
						else
						{
							xResult = W_xResult;
						}
						
						// Test
						xResult = W_xResult;
					}
				}
				else
				{
					//xResult = M_xResult;
					//yResult = M_yResult;
					
					xResult = W_xResult;
					yResult = W_yResult;
				}
			}
			
		
		// Verify the heading angle
			// white = 0, black = 2, grey = 1
	        if ((lineSensorLeft == 0) && (lineSensorRight == 0))		// robot moves on the line
	        {	
	        	if ((W_aResult <= TRSH_B) || (W_aResult <= (TRSH_B - 360)))		// mozna bude nutne zmenit podminku
	        	{
	        		aVerif[0] = 1;
	        	}
	        	else
	        	{
	        		aVerif[0] = 0;
	        	}
	        	
	        	if ((M_aResult <= TRSH_B) || (M_aResult <= (TRSH_B - 360)))
	        	{
	        		aVerif[1] = 1;
	        	}
	        	else
	        	{
	        		aVerif[1] = 0;
	        	}
			}
	        // robot moves left or right from the line
			else if(((lineSensorLeft == 0) && (lineSensorRight == 2)) || ((lineSensorLeft == 2) && (lineSensorRight == 0)))	
			{
	        	if ((W_aResult <= TRSH_W) || (W_aResult <= (TRSH_W - 360)))
	        	{
	        		aVerif[0] = 1;
	        	}
	        	else
	        	{
	        		aVerif[0] = 0;
	        	}
	        	
	        	if ((M_aResult <= TRSH_W) || (M_aResult <= (TRSH_W - 360)))
	        	{
	        		aVerif[1] = 1;
	        	}
	        	else
	        	{
	        		aVerif[1] = 0;
	        	}
			}
	        // robot moves left or right to the grey area
	        else if(((lineSensorLeft == 1) && (lineSensorRight == 0)) || ((lineSensorLeft == 0) && (lineSensorRight == 1)))	
	        {
	        	if ((W_aResult <= TRSH_G) || (W_aResult <= (TRSH_G - 360)))
	        	{
	        		aVerif[0] = 1;
	        	}
	        	else
	        	{
	        		aVerif[0] = 0;
	        	}
	        	
	        	if ((M_aResult <= TRSH_G) || (M_aResult <= (TRSH_G - 360)))
	        	{
	        		aVerif[1] = 1;
	        	}
	        	else
	        	{
	        		aVerif[1] = 0;
	        	}
			}
			
			// Evaluation of the angle
			if ((aVerif[0] == 0) || (aVerif[1] == 1))
			{
				aResult = M_aResult;
			}
			else if ((aVerif[0] == 1) || (aVerif[1] == 0))
			{
				aResult = W_aResult;
			}
			else if  ((aVerif[0] == 0) || (aVerif[1] == 0))
			{
				// Vymyslet co tady - ani jeden uhel nebyl verifikovan
				aResult = (W_aResult + M_aResult)/2;
			}
			else
			{
				aResult = (W_aResult + M_aResult)/2;
			}
			
			//xResult = W_xResult;
			//yResult = W_yResult;
			aResult = W_aResult;
			
		// Return pose	
		this.pose.setLocation((float)xResult, (float)yResult);
		this.pose.setHeading((float)aResult);		 
	}

	/**
	 * detects parking slots and manage them by initializing new slots, re-characterizing old slots or merge old and detected slots. 
	 */
	private void detectParkingSlot() // Neni dokonceno vyhodnocovani minulych slotu - jeden bude ulozen vicekrat
	{		
		Point PosS = new Point(0,0);
		Point PosE = new Point(0,0);
		
		double sum_F = 0;
		double sum_B = 0;
		
		double distance_F = 0;
		double distance_B = 0;
		
		int SlotID = counter;
		INavigation.ParkingSlot.ParkingSlotStatus SlotStatus = ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING;
		
		short axe = getHeadingAxe();
		
		for (int i = 1; i <= 4; i++)
		{
			DIST_FS[i] = DIST_FS[i-1];
			sum_F = DIST_FS[i] + sum_F;
			
			DIST_BS[i] = DIST_BS[i-1];
			sum_B = DIST_BS[i] + sum_B;
		}
		
		//DIST_FS[0] = frontSideSensorDistance;
		DIST_FS[0] = frontSensorDistance;
		distance_F = (sum_F + DIST_FS[0])/5;
		
		DIST_BS[0] = backSideSensorDistance;
		distance_B = (sum_B + DIST_BS[0])/5;
		
		// Saving the begin point of the PS
		if ((distance_F <= TRSH_SG) && (burstFS == 0))
		{
			PosF1.setLocation(this.pose.getX(), this.pose.getY());
			burstFS = 1;
			burstFE = 0;
		}
		
		if ((distance_B <= TRSH_SG) && (burstRS == 0))
		{
			PosR1.setLocation(this.pose.getX(), this.pose.getY());
			burstRS = 1;
			burstRE = 0;
		}
				
		// Saving the end point of the PS
		if ((distance_F >= TRSH_SG) && (burstFE == 0))
		{
			PosF2.setLocation(this.pose.getX(), this.pose.getY());
			burstFS = 0;
			burstFE = 1;
		}
		
		if ((distance_B >= TRSH_SG) && (burstRE == 0))
		{
			PosR2.setLocation(this.pose.getX(), this.pose.getY());
			burstRS = 0;
			//burstRE = 1;
		}
		
		if ((burstRS == 0) && (burstRE == 0) && (counter < 10))
		{
			PosS.setLocation(((PosF1.getX() + PosR1.getX())/2), ((PosF1.getY() + PosR1.getY())/2));
			PosE.setLocation(((PosF2.getX() + PosR2.getX())/2), ((PosF2.getY() + PosR2.getY())/2));
			
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
			
			slotList[counter++] = new INavigation.ParkingSlot(SlotID, PosE, PosS, SlotStatus, 0);
			
			burstRE = 1;
		}
		
		return; // has to be implemented by students
	}
}