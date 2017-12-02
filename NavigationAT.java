package parkingRobot.hsamr0;

import lejos.geom.Line;
import lejos.robotics.navigation.Pose;

import parkingRobot.INavigation;
import parkingRobot.IPerception;
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
	static final double LEFT_WHEEL_RADIUS	= 	0.00281; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: radius of right wheel
	 */
	static final double RIGHT_WHEEL_RADIUS	= 	0.002765; // only rough guess, to be measured exactly and maybe refined by experiments
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
	double W_deltaX		= 0;
	double W_deltaY		= 0;
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
	double[] DELTA_X = {0,0,0,0,0};
	double[] DELTA_Y = {0,0,0,0,0};
	
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
	
	static final double TRSH_SG = 0;
	static final double FGS_Dist_P = 0;
	static final double RGS_Dist_P = 0;
	
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
			this.W_xResult		= this.pose.getX();
			this.W_yResult		= this.pose.getY();
			this.W_aResult 		= this.pose.getHeading();
		} 
		else if (R.isInfinite()) //robot moves straight forward/backward, vLeft==vRight
		{ 
			this.W_xResult		= this.pose.getX() + vLeft * Math.cos(this.pose.getHeading()) * deltaT;
			this.W_yResult		= this.pose.getY() + vLeft * Math.sin(this.pose.getHeading()) * deltaT;
			this.W_aResult		= this.pose.getHeading();
		} 
		else 
		{			
			ICCx = this.pose.getX() - R.doubleValue() * Math.sin(this.pose.getHeading());
			ICCy = this.pose.getY() + R.doubleValue() * Math.cos(this.pose.getHeading());
		
			this.W_xResult 		= Math.cos(w * deltaT) * (this.pose.getX()-ICCx) - Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
			this.W_yResult 		= Math.sin(w * deltaT) * (this.pose.getX()-ICCx) + Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
			this.W_aResult 		= this.pose.getHeading() + w * deltaT;
		}
		
		this.W_deltaX = Math.abs(this.W_xResult - this.pose.getX());
		this.W_deltaY = Math.abs(this.W_yResult - this.pose.getY());
		
		this.W_dist	 = Math.sqrt(Math.pow(this.W_deltaX, 2) + Math.pow(this.W_deltaY, 2));	// Driven distance - wheel odometry
		//W_CaRes  = Math.atan(W_dist/W_deltaX);						// Counted heading angle - wheel odometry ----- mozna neni potreba
	}
	
	/**
	 *  calculates the mouse odometry
	 */
	private void mauseOdometrie()
	{
		this.M_deltaX = this.mouseOdoMeasurement.getUSum()/1000;		// dostanu deltaX, deltaY a deltaT v m
		this.M_deltaY = this.mouseOdoMeasurement.getVSum()/1000;
		
		this.M_xResult = this.M_deltaX + this.M_xPosOld;
		this.M_yResult = this.M_deltaY + this.M_yPosOld;
		
		this.M_xPosOld = this.M_xResult;
		this.M_yPosOld = this.M_yResult;
		
		this.M_dist	 = Math.sqrt(Math.pow(this.M_deltaX, 2) + Math.pow(this.M_deltaY, 2));	// Driven distance - mouse sensor
		this.M_aResult  = Math.atan(this.M_deltaY/this.M_deltaX);							// Counted heading angle - mouse sensor
	}
	
	/**
	 * calculates the axe of the movement
	 */
	private short getHeadingAxe()
	{
		double avgX = 0;
		double avgY = 0;
		
		double sumX = 0;
		double sumY = 0;
		
		short movDir = 0;
		
		for (int i = 1; i <= 5; i++)
		{
			this.DELTA_X[i] = this.DELTA_X[i-1];
			sumX = this.DELTA_X[i] + sumX;
			
			this.DELTA_Y[i] = this.DELTA_Y[i-1];
			sumY = this.DELTA_Y[i] + sumY;
		}
		
		this.DELTA_X[0] = this.pose.getX();
		avgX = (sumX + this.DELTA_X[0])/5;
		
		this.DELTA_Y[0] = this.pose.getY();
		avgY = (sumY + this.DELTA_Y[0])/5;
		
		if (avgX > avgY)
		{
			movDir = 0;
		}
		else if (avgX < avgY)
		{
			movDir = 1;
		}
		else
		{
			movDir = 2;
		}
		
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
		
		// Potom smazat !!!!!!!!!!!!!!
		double xResult = 0;
		double yResult = 0;
		
		for (int i = 1; i <= 5; i++)
		{
			this.DIST_F[i] = this.DIST_F[i-1];
			sum_F = DIST_F[i] + sum_F;
			
			this.DIST_B[i] = this.DIST_B[i-1];
			sum_B = DIST_B[i] + sum_B;
		}
		
		this.DIST_F[0] = this.frontSensorDistance;
		distance_F = (sum_F + this.DIST_F[0])/5;
		
		this.DIST_B[0] = this.backSensorDistance;
		distance_B = (sum_B + this.DIST_B[0])/5;
		
		if (this.CORNER_ID == 4)
		{
			if (distance <= TRSH_DIST4)
			{
				corner = 1;
				burst = 1;
				
				xResult = 150;
				yResult = 30;
			}
			else
			{
				corner = 0;
			}
		}
		else if (this.CORNER_ID == 5)
		{
			if (distance <= TRSH_DIST5)
			{
				corner = 1;
				burst = 1;
				
				xResult = 30;
				yResult = 30;
			}
			else
			{
				corner = 0;
			}
		}
		else
		{
			if (distance <= TRSH_DISTN)
			{
				corner = 1;
				burst = 1;
				
				switch (this.CORNER_ID)
				{
				case 0:
					xResult = 0;
					yResult = 0;
					
					this.M_xPosOld = 0;
					this.M_yPosOld = 0;
					break;
				case 1:
					xResult = 180;
					yResult = 0;
					break;
				case 2:
					xResult = 180;
					yResult = 60;
					break;
				case 3:
					xResult = 150;
					yResult = 60;
					break;
				case 6:
					xResult = 30;
					yResult = 60;
					break;
				case 7:
					xResult = 0;
					yResult = 60;
					break;
				}
			}
			else
			{
				corner = 0;
			}
			
			return corner;
		}
		
		if (burst == 1)			// Incrementing the corner
		{
			burst = 0;
			
			if (this.CORNER_ID < 8)
			{
				this.CORNER_ID++;
			}
			else
			{
				this.CORNER_ID = 0;
			}
		}
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
		short corn = 0;
		
		// Wheel odometry
		this.radOdometrie();
		
		// Mouse sensor
		this.mauseOdometrie();
		
		// Data evaluation
			
			// Detecting the axe of the movement
			axe = this.getHeadingAxe();
			
			// Detecting the corners
			corn = this.getCorner();
			
			// Verify the coordinates
			if ((this.lineSensorLeft == 0) && (this.lineSensorRight == 0))
			{
				if (axe == 0)		// movement in x direction
				{
					xResult = this.M_xResult;
					
					switch (this.CORNER_ID)
					{
					case 0: yId = 0; break;
					case 2:
					case 6: yId = 60; break;
					case 4: yId = 30; break;
					}
					
					yDiff_W = Math.abs(this.W_yResult - yId);
					yDiff_M = Math.abs(this.M_yResult - yId);
					
					if (yDiff_M < = yDiff_W)
					{
						yResult = this.M_yResult;
					}
					else
					{
						yResult = this.W_yResult;
					}
				}
				else if (axe == 1)	// movement in y direction
				{
					yresult = this.M_yResult;
					
					switch (this.CORNER_ID)
					{
					case 1: xId = 180; break;
					case 3: xId = 150; break;
					case 5: xId = 30; break;
					case 7: xId = 0; break;
					}
					
					xDiff_W = Math.abs(this.W_xResult - xId);
					xDiff_M = Math.abs(this.M_xResult - xId);
					
					if (xDiff_M < = xDiff_W)
					{
						xResult = this.M_xResult;
					}
					else
					{
						xResult = this.W_xResult;
					}
				}
				else
				{
					xResult = this.M_xResult;
					yResult = this.M_yResult;
				}
			}
			else if(((this.lineSensorLeft == 0) && (this.lineSensorRight == 2)) || ((this.lineSensorLeft == 2) && (this.lineSensorRight == 0)))
			{
				// presunout kod zeshora, nahore rovnou zapisovat idealni koordinaty
			}
			else
			{
				xResult = this.M_xResult;
				yResult = this.M_yResult;
			}
		
		// Verify the heading angle

			// white = 0, black = 2, grey = 1
	        if ((this.lineSensorLeft == 0) && (this.lineSensorRight == 0))		// robot moves on the line
	        {	
	        	if ((this.W_aResult <= TRSH_B) || (this.W_aResult <= (TRSH_B - 360)))		// mozna bude nutne zmenit podminku
	        	{
	        		aVerif[0] = 1;
	        	}
	        	else
	        	{
	        		aVerif[0] = 0;
	        	}
	        	
	        	if (this.(M_aResult <= TRSH_B) || (this.M_aResult <= (TRSH_B - 360)))
	        	{
	        		aVerif[1] = 1;
	        	}
	        	else
	        	{
	        		aVerif[1] = 0;
	        	}
			}
	        // robot moves left or right from the line
			else if(((this.lineSensorLeft == 0) && (this.lineSensorRight == 2)) || ((this.lineSensorLeft == 2) && (this.lineSensorRight == 0)))	
			{
	        	if ((this.W_aResult <= TRSH_W) || (this.W_aResult <= (TRSH_W - 360)))
	        	{
	        		aVerif[0] = 1;
	        	}
	        	else
	        	{
	        		aVerif[0] = 0;
	        	}
	        	
	        	if ((this.M_aResult <= TRSH_W) || (this.M_aResult <= (TRSH_W - 360)))
	        	{
	        		aVerif[1] = 1;
	        	}
	        	else
	        	{
	        		aVerif[1] = 0;
	        	}
			}
	        // robot moves left or right to the grey area
	        else if(((this.lineSensorLeft == 1) && (this.lineSensorRight == 0)) || ((this.lineSensorLeft == 0) && (this.lineSensorRight == 1)))	
	        {
	        	if ((this.W_aResult <= TRSH_G) || (this.W_aResult <= (TRSH_G - 360)))
	        	{
	        		aVerif[0] = 1;
	        	}
	        	else
	        	{
	        		aVerif[0] = 0;
	        	}
	        	
	        	if ((this.M_aResult <= TRSH_G) || (this.M_aResult <= (TRSH_G - 360)))
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
				aResult = this.M_aResult;
			}
			else if ((aVerif[0] == 1) || (aVerif[1] == 0))
			{
				aResult = this.W_aResult;
			}
			else if  ((aVerif[0] == 0) || (aVerif[1] == 0))
			{
				// Vymyslet co tady - ani jeden uhel nebyl verifikovan
				aResult = (this.W_aResult + this.M_aResult)/2;
			}
			else
			{
				aResult = (this.W_aResult + this.M_aResult)/2;
			}

		// Return pose	
		this.pose.setLocation((float)xResult, (float)yResult);
		this.pose.setHeading((float)aResult);		 
	}

	/**
	 * detects parking slots and manage them by initializing new slots, re-characterizing old slots or merge old and detected slots. 
	 */
	private void detectParkingSlot() // Neni dokoncena, pouvazovat
	{
		double xPosF1 = 0;
		double yPosF1 = 0;
		double xPosR1 = 0;
		double yPosR1 = 0;
		
		double xPosF2 = 0;
		double yPosF2 = 0;
		double xPosR2 = 0;
		double yPosR2 = 0;
		
		double xPos1 = 0;
		double yPos1 = 0;
		double xPos2 = 0;
		double yPos2 = 0;
		
		double pPosX = 0;
		double pPosY = 0;
		
		double FGS_Dist = perception.getFrontSideSensorDistance();
		double RGS_Dist = perception.getBackSideSensorDistance();
		
		short SlotStatus = 0;
		short axe = 0;
		
		short burstFS = 0;
		short burstBS = 0;
		short burstFE = 0;
		short burstBE = 0;
		
		axe = this.getHeadingAxe();
		
		for (int i = 1; i <= 5; i++)
		{
			this.DIST_FS[i] = this.DIST_FS[i-1];
			sum_F = DIST_FS[i] + sum_F;
			
			this.DIST_BS[i] = this.DIST_BS[i-1];
			sum_B = DIST_BS[i] + sum_B;
		}
		
		this.DIST_FS[0] = this.frontSideSensorDistance;
		distance_F = (sum_F + this.DIST_FS[0])/5;
		
		this.DIST_BS[0] = this.backSideSensorDistance;
		distance_B = (sum_B + this.DIST_BS[0])/5;
		
		// Saving the begin point of the PS
		if ((distance_F >= TRSH_SG) && (burstFS == 0))
		{
			xPosF1 = this.pose.getX();
			yPosF1 = this.pose.getY();
			
			burstFS = 1;
		}
		
		if ((distance_B >= TRSH_SG) && (burstBS == 0))
		{
			xPosR1 = this.pose.getX();
			yPosR1 = this.pose.getY();
			
			burstBS = 1;
		}
				
		// Saving the end point of the PS
		// pridat burst
		if ((distF >= TRSH_SG) && (burstFE == 0)
		{
			xPosF2 = this.pose.getX();
			yPosF2 = this.pose.getY();
			
			burstFE = 1;
		}
		
		if ((distB >= TRSH_SG) && (burstBE == 0))
		{
			xPosR2 = this.pose.getX();
			yPosR2 = this.pose.getY();
			
			xPos1 = (xPosF1+xPosR1)/2;
			yPos1 = (yPosF1+yPosR1)/2;
			
			xPos2 = (xPosF2+xPosR2)/2;
			yPos2 = (yPosF2+yPosR2)/2;
			
			// Evaluatin of the slot
			if (axe == 0)
			{
				if ((xPos2-xPos1) > LGNT_ROBOT)
				{
					SlotStatus = 1;
				}
				else
				{
					SlotStatus = 0;
				}
			}
			else
			{
				if ((yPos2-yPos1) > LGNT_ROBOT)
				{
					SlotStatus = 1;
				}
				else
				{
					SlotStatus = 0;
				}
			}
			
			burstBE = 1;
		}
		
		return; // has to be implemented by students
	}
}