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
	static final double LEFT_WHEEL_RADIUS	= 	2.81; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: radius of right wheel
	 */
	static final double RIGHT_WHEEL_RADIUS	= 	2.765; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: distance between wheels
	 */
	static final double WHEEL_DISTANCE		= 	151.1; // only rough guess, to be measured exactly and maybe refined by experiments

	/**
	 * data fusion constants and variables
	 */
	// Mause sensor
	static final double M_xPosOld = 0;
	static final double M_yPosOld = 0;
	
	// Corner detection
	double DIST[5];
	
	int CORNER_ID = 0;
	int CORNER = 0;
	
	// Angle verification
	static final double TRSH_B = 0;
	static final double TRSH_W = 0;
	static final double TRSH_G = 0;
	
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
	public synchronized ParkingSlot[] getParkingSlots() {
		return null;
	}
	
	
	// Private methods
	
	/**
	 * calls the perception methods for obtaining actual measurement data and writes the data into members
	 */
	private void updateSensors(){		
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
	 * calculates the robot pose from the measurements
	 */
	private void calculateLocation()
	{
		double leftAngleSpeed 	= this.angleMeasurementLeft.getAngleSum()  / ((double)this.angleMeasurementLeft.getDeltaT()/1000);  //degree/seconds
		double rightAngleSpeed 	= this.angleMeasurementRight.getAngleSum() / ((double)this.angleMeasurementRight.getDeltaT()/1000); //degree/seconds

		double vLeft	= (leftAngleSpeed  * Math.PI * LEFT_WHEEL_RADIUS ) / 180 ; //velocity of left  wheel in m/s
		double vRight	= (rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180 ; //velocity of right wheel in m/s		
		double w 		= (vRight - vLeft) / WHEEL_DISTANCE; //angular velocity of robot in rad/s
		
		Double R 		= new Double(( WHEEL_DISTANCE / 2 ) * ( (vLeft + vRight) / (vRight - vLeft) ));
		double deltaT   = ((double)this.angleMeasurementLeft.getDeltaT())/1000;
		
		double ICCx 	= 0;
		double ICCy 	= 0;

		double W_xResult 	= 0;
		double W_yResult 	= 0;
		double W_aResult	= 0;		
		double W_deltaX		= 0;
		double W_deltaY		= 0;
		double W_dist		= 0;
		
		double M_xResult 	= 0;
		double M_yResult 	= 0;
		double M_aResult	= 0;	
		double M_deltaX		= 0;
		double M_deltaY		= 0;
		double M_dist		= 0;
		
		short aVerif 	= [0 0];
		short burst 	= 0;
		
		double xResult 	= 0;
		double yResult 	= 0;
		double aResult	= 0;
		
		double FWS_Dist = perception.getFrontSensorDistance();
		double FGS_Dist = perception.getFrontSideSensorDistance();
		double RWS_Dist = perception.getBackSensorDistance();
		double RGS_Dist = perception.getBackSideSensorDistance();
		double distance = 0;
		
		// Wheel odometry
		
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
		
		W_deltaX = Math.abs(W_xResult - this.pose.getX());
		W_deltaY = Math.abs(W_yResult - this.pose.getY());
		
		W_dist	 = sqrt(W_deltaX^2 + W_deltaY^2);			// Driven distance - wheel odometry
		W_CaRes  = Math.tan(W_deltaY, W_deltaX);			// Counted heading angle - wheel odometry ----- mozna neni potreba
		
		// Mouse sensor
		
		/* DOPLNIT MYSI SENSOR */
		M_deltaX = this.UOdometry();		// dostanu deltaX, deltaY a deltaT
		M_deltaY = this.VOdometry();
		
		M_xResult = M_deltaX + M_xPosOld;
		M_yResult = M_deltaY + M_yPosOld;
		
		M_xPosOld = M_xResult;
		M_yPosOld = M_yResult;
		
		M_dist	 = sqrt(M_xIncrement^2 + M_yIncrement^2);	// Driven distance - mouse sensor
		M_aResult  = Math.tan(M_deltaY, M_deltaX);			// Counted heading angle - mouse sensor
		
		// Evaluation of coordinates
		
		// Doplnit opravdovym kodem
		xResult = W_xResult;
		yResult = W_yResult;
		
		// Detecting the corners
		
		//pridat vypocet vzdalenosti (plovouci prumer, filtrovane data...), mozna pridat i odhad souradnic v rozich
		for (int i = 1; i <= 5; i++)
		{
			DIST[i] = DIST[i-1];
			sum = DIST[i] + sum;
		}
		DIST[0] = FWS_Dist;
		distance = (sum + DIST[0])/5;
		
		
		if (CORNER_ID == 4)
		{
			if (distance <= TRSH_DIST4)
			{
				CORNER = 1;
				burst = 1;
				
				xResult = 150;
				yResult = 30;
			}
			else
			{
				CORNER = 0;
			}
		}
		else if (CORNER_ID == 5)
		{
			if (distance <= TRSH_DIST5)
			{
				CORNER = 1;
				burst = 1;
				
				xResult = 30;
				yResult = 30;
			}
			else
			{
				CORNER = 0;
			}
		}
		else
		{
			if (distance <= TRSH_DISTN)
			{
				CORNER = 1;
				burst = 1;
				
				switch (CORNER_ID)
				{
				case 0:
					xResult = 0;
					yResult = 0;
					
					M_xPosOld = 0;
					M_yPosOld = 0;
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
				CORNER = 0;
			}
		}
		
		if (burst == 1)			// Incrementing the corner
		{
			burst = 0;
			
			if (CORNER_ID < 8)
			{
				CORNER_ID++;
			}
			else
			{
				CORNER_ID = 0;
			}
		}
		
		// Verify the heading angle ------------------- Mozna neni treba verifikovat pouze mimo rohy
		if (CORNER == 0)
		{
			// white = 0, black = 2, grey = 1
	        if ((this.lineSensorLeft == 0) && (this.lineSensorRight == 0))		// robot moves on the line
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
			else if(((this.lineSensorLeft == 0) && (this.lineSensorRight == 2)) || ((this.lineSensorLeft == 2) && (this.lineSensorRight == 0)))	
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
	        else if(((this.lineSensorLeft == 1) && (this.lineSensorRight == 0)) || ((this.lineSensorLeft == 0) && (this.lineSensorRight == 1)))	
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

		// Return pose	
		this.pose.setLocation((float)xResult, (float)yResult);
		this.pose.setHeading((float)aResult);		 
	}

	/**
	 * detects parking slots and manage them by initializing new slots, re-characterizing old slots or merge old and detected slots. 
	 */
	private void detectParkingSlot()
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
		
		// Saving the begin point of the PS
		// pridat burst
		if ((FGS_Dist >= TRSH_SG) && (FGS_Dist != FGS_Dist_P))
		{
			xPosF1 = this.pose.getX();
			yPosF1 = this.pose.getY();
		}
		
		if ((RGS_Dist >= TRSH_SG) && (RGS_Dist != RGS_Dist_P))
		{
			xPosR1 = this.pose.getX();
			yPosR1 = this.pose.getY();
		}
		
		xPos1 = (xPosF1+xPosR1)/2;
		yPos1 = (yPosF1+yPosR1)/2;
		
		// Saving the end point of the PS
		// pridat burst
		if ((FGS_Dist <= TRSH_SG) && (FGS_Dist != FGS_Dist_P))
		{
			xPosF2 = this.pose.getX();
			yPosF2 = this.pose.getY();
		}
		
		if ((RGS_Dist <= TRSH_SG) && (RGS_Dist != RGS_Dist_P))
		{
			xPosR2 = this.pose.getX();
			yPosR2 = this.pose.getY();
		}
		
		xPos2 = (xPosF2+xPosR2)/2;
		yPos2 = (yPosF2+yPosR2)/2;
		
		// Evaluatin of the slot
		// pridat pro parkovani v y a dodelat
		if ((xPos2-xPos1) > LGNT_ROBOT)
		{
			// add to object the first and last point, length of the slot and the slot ID
		}
		
		return; // has to be implemented by students
	}
}