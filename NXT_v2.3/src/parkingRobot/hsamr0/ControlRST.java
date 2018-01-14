package parkingRobot.hsamr0;


import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.nxt.NXTMotor;
import parkingRobot.INavigation;
//import lejos.nxt.comm.RConsole;



/**
 * Main class for control module
 *
 */

public class ControlRST implements IControl {
	
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
	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorLeft	=	0;
	
	NXTMotor leftMotor = null;
	NXTMotor rightMotor = null;
	
	IPerception perception = null;
	INavigation navigation = null;
	IMonitor monitor = null;
	ControlThread ctrlThread = null;

    int leftMotorPower = 0;
	int rightMotorPower = 0;
	
	Pose startPosition = new Pose();
	Pose currentPosition = new Pose();
	Pose destination = new Pose();
	
	ControlMode currentCTRLMODE = null;
	
	
	EncoderSensor controlRightEncoder    = null;
	EncoderSensor controlLeftEncoder     = null;

	int lastTime = 0;
	int lowPower = 1;
	int highPower =20;
    double currentDistance = 0.0;
    double Distance = 0.0;
    double L_controlSpeed=0.0;
    double Leftspeed=0.0;
    int i=0;
    
    
  //start PID Controler    
    PidC pidLineFollow1 = new PidC();
    PidC PidVwControl1 = new PidC();
    PidC PidVwControl2 = new PidC();
    PidC PidVwControl3 = new PidC();
    PidC pidSetPose1 = new PidC();
    PidC pidSetPose2 = new PidC();
    
  //start linefollow added variable
    int statusModus=0; //1=LINE_CTRL, 2=VW_CTRL, 3=SETPOSE, 4=PARK_CTRL, INACTIVE
	int switchModeLineFollow=2;		// 1=Binary; 2=PID-Control; 3=PID-mit Curve-Control
  
	int rel=0;
	int lineSensorRightValue	=	0;
	int lineSensorLeftValue		=	0;
	double X = 0;
	double w = 0;
	double controlOut = 0;
	double controlVar=0;
	double deltvaule=0;
	double Wtarge=0;
	double deltvauleInt=0;
	double deltvauleLast=0;
	// int flag0=0;   //flag=0:v/wcontrol  Koeffizient
	//end linefollow added variable
	
	//start v/w added variable
		double dTime;
		double leftSpeed;
		double rightSpeed;
		double wheelDiameter = 5.6;//cm
		double radius;
		double radiusL;
		double radiusR;
		double trackWidth = 15.5;
		double distancePerTurn = Math.PI*wheelDiameter;
		double distancePerDeg = distancePerTurn/360;
		double velocityLeft;
		double velocityRight;
		double velocity = 80;              //47 10
		double angularVelocity=0;     //0.5 entspreche 2.1-Drehung um 90Â° in math. pos. Richtung;15Â°/s ;4.0-Drehung um 90Â° in math. neg. Richtung
		double MVangularVelocity;
		double MVvelocity;
		int leftPower=0;
		int rightPower=0;
		double rightAngSpeed=0;
		double leftAngSpeed=0;
		double S=0;
	    int Drehungrichtung=2;    //0:negativ;1:positiv;2:close Drehung
		//public static final double addAngle = 0;
		//end v/w added variable
	    
	  //start SetPose added variable
	    double heading=0; //setDestination
	    double x=50;
	    double y=0;
	    double x0;
    	double y0;
    	double angle0;
    	double x1;
    	double y1;
    	double angle1;
    	double  deltx;
    	double  delty;
    	double a4;
    	double alpha1;
    	double S0;
    	int flag=0;
    	int flag0=0;
	    int flag2=0;
    	int temp=0;
    	//double a2;
    	double alpha2;
    	double newX;
    	double newY;
    	double X1;
    	double Vergleichvalue=0 ;
    	
	  //end SetPose added variable
    	
     //start PARKCTRL added variable
    	int    flag5=1;    //flag5 -1：zuruck  1:hin 
		double s;
		double dots;
        double t=0;
		double x1a ;    //Anfangsposition f黵 x1
		double x2a ;    //Anfangsposition f黵 x2
		double x1b; //Endposition f黵 x1
		double x2b; //Endposition f黵 x2
		double ki;   //Anfangsgeschwindigkeit
		double kf;    //Endgeschwindigkeit
		//Parameter f黵 Polynome x1 und x2
		double a0 ;
		double a1;
		double a2 ;
		double a3;
		double b0 ;
		double b1;
		double b2 ;
		double b3;
  		double T; //Zeit f黵 die gesamten Fahrt
  		double thetai;
  		double thetaf; 
  		
     //end PARKCTRL added variable
    	
	/**
	 * provides the reference transfer so that the class knows its corresponding navigation object (to obtain the current 
	 * position of the car from) and starts the control thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param navigation corresponding main module Navigation class object
	 * @param monitor corresponding main module Monitor class object
	 * @param leftMotor corresponding NXTMotor object
	 * @param rightMotor corresponding NXTMotor object
	 */
	public ControlRST(IPerception perception, INavigation navigation, NXTMotor leftMotor, NXTMotor rightMotor, IMonitor monitor){
		this.perception = perception;
        this.navigation = navigation;
        this.monitor = monitor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		this.currentCTRLMODE = ControlMode.INACTIVE;//ControlMode.SETPOSE ;  //switch Ctrmodul veraendert
		//angle difference between actual an last request
		this.encoderLeft  = perception.getControlLeftEncoder();
		this.encoderRight = perception.getControlRightEncoder();
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		
		// MONITOR (example)
		monitor.addControlVar("RightSensor");
		monitor.addControlVar("LeftSensor");
		
		this.ctrlThread = new ControlThread(this);
		
		ctrlThread.setPriority(Thread.MAX_PRIORITY - 1);
		ctrlThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		ctrlThread.start();
	}
	

	// Inputs
	
	/**
	 * set velocity
	 * @see parkingRobot.IControl#setVelocity(double velocity)
	 */
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	/**
	 * set angular velocity
	 * @see parkingRobot.IControl#setAngularVelocity(double angularVelocity)
	 */
	public void setAngularVelocity(double angularVelocity) {
		this.angularVelocity = angularVelocity;

	}
	
	public void setDestinationheading(double heading) {
		this.heading = heading;
	}

	public void setDestinationx(double x) {
		this.x = x;

	}
	
	public void setDestinationy(double y) {
		this.y = y;

	}
	/**
	 * sets current pose
	 * @see parkingRobot.IControl#setPose(Pose currentPosition)
	 */
	public void setPose(Pose currentPosition) {
		// TODO Auto-generated method stub
		this.currentPosition = currentPosition;
	}
	
	
	/**
	 * set destination
	 * @see parkingRobot.IControl#setDestination(double heading, double x, double y)
	 */
	public void setDestination(double heading, double x, double y){
		this.destination.setHeading((float) heading);
		this.destination.setLocation((float) x, (float) y);
	}
	


	/**
	 * set control mode
	 */
	
	public void setCtrlMode(ControlMode ctrl_mode) {
		this.currentCTRLMODE = ctrl_mode;
	}
	
	/**
	 * set start time
	 */
	public void setStartTime(int startTime){
		this.lastTime = startTime;
	}
	
	/**
	 * selection of control-mode
	 * @see parkingRobot.IControl#exec_CTRL_ALGO()
	 */
	
	public void exec_CTRL_ALGO(){
		
		switch (currentCTRLMODE)
		{
		  case   SETPOSE   : update_LINECTRL_Parameter();
		                      exec_LINECTRL_ALGO();
		                      break;
		  case   VW_CTRL  :   update_VWCTRL_Parameter();
		   					  exec_VWCTRL_ALGO();
		   					  break; 
		  case PARK_CTRL     : update_SETPOSE_Parameter();
			  				  exec_SETPOSE_ALGO();
		                      break;
		  case LINE_CTRL  :   update_PARKCTRL_Parameter();
		  					  exec_PARKCTRL_ALGO();
		  					  break;		  					  
		  case INACTIVE 	: exec_INACTIVE();
			                  break;
		}

	}
	
	// Private methods
	
	/**
	 * update parameters during VW Control Mode
	 */
	private void update_VWCTRL_Parameter(){
		this.encoderRight  = perception.getControlRightEncoder();
	    this.encoderLeft  = perception.getControlLeftEncoder();
		setPose(navigation.getPose());
	}
    private void exec_VWCTRL_ALGO(){  
				this.drive(this.velocity, this.angularVelocity);
     }
	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter(){
		this.encoderRight  = perception.getControlRightEncoder();
	    this.encoderLeft  = perception.getControlLeftEncoder();
		setPose(navigation.getPose());
	}
	
	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter(){
		//Aufgabe 3.4
		this.encoderRight  = perception.getControlRightEncoder();
		this.encoderLeft  = perception.getControlLeftEncoder();
		setPose(navigation.getPose());
	}

	/**
	 * update parameters during LINE Control Mode
	 */
	private void update_LINECTRL_Parameter(){
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();	
		//0-100; 0-black 100-white
		this.lineSensorRightValue		= perception.getRightLineSensorValue();
		this.lineSensorLeftValue  		= perception.getLeftLineSensorValue();
		setPose(navigation.getPose());
		this.encoderRight  = perception.getControlRightEncoder();
		this.encoderLeft  = perception.getControlLeftEncoder();
	}
	
	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade during VW Control Mode
	 * optionally one of them could be set to zero for simple test.
	 */
	
    private void exec_SETPOSE_ALGO(){
    	//leftMotor.forward();
		//rightMotor.forward();
    	//Aufgabe 3.3
    	//step 1ï¼šin first quadrant ,Originposition as the starting point
    	switch (flag2) {
		case 0:      
		   if(temp==0) {
			
     	          x0=0;            
    	          y0=0;             
    	        angle0=currentPosition.getHeading();            
    	         x1=120;                      
    	         y1=0;                     //destination.getY();
    	         angle1=0;             //destination.getHeading();
		       }
		   else if(temp==1) {
			
	     	       x0=120;           
	    	      y0=0;            
	    	       angle0=currentPosition.getHeading();             
	    	      x1=120;                      //destination.getX();
	    	      y1=30;                     //destination.getY();
	    	      angle1=0;             //destination.getHeading();
	    	    if(i==0) Vergleichvalue=currentPosition.getX()*100;
	    	         i=1;
			        }
    	deltx=x1-x0;
    	delty=y1-y0;
        S0= (float)(Math.sqrt(Math.pow(deltx,2)+Math.pow(delty,2)));
                  if(deltx!=0) {
                       a4=(float)(delty/deltx);  //in first quadrant ,Originposition as the starting point   Summe von Winkel
    	               alpha1=Math.atan(a4);   //phi
                 }
                   else{
                   	alpha1=Math.PI/2;
                     }
    	newX=(float)((navigation.getPose().getX())*100*Math.cos(alpha1)+(navigation.getPose().getY())*100*Math.sin(alpha1));
    	newY=(float)((navigation.getPose().getY())*100*Math.cos(alpha1)-(navigation.getPose().getX())*100*Math.sin(alpha1));
    	if(temp==1) {
    		newY=-newY;
    	}
        if (flag==0) {
        	velocity=30;
	    	pidSetPose2.setParameter(8, 0, 80, 0);  //8 0 80 0
        	angularVelocity=-pidSetPose2.getY(Vergleichvalue,newY);
        	if (temp==1) {
        		angularVelocity=-angularVelocity;
        	}
        	/**        	if(angularVelocity<=-2.4) {
        		angularVelocity=-2.4;
    		}
    		else if (angularVelocity>=2.4) {
    			angularVelocity=2.4;
    		} */
	    	update_VWCTRL_Parameter();
		    exec_VWCTRL_ALGO(); 
    	}
    	else if (flag==1) {
    		   if(flag0==0) { 
    		      velocity=0;
    		     angularVelocity=2.1;
    		     update_VWCTRL_Parameter();
    			 exec_VWCTRL_ALGO();
    		   }
    		   else if (flag0==1) {
    	    		 pidSetPose2.setParameter(50, 0, 0, 0);
    		         if(Math.abs(currentPosition.getHeading()-Math.PI/2)>0.05236/4) {
    		        	 angularVelocity=-pidSetPose2.getY(Math.PI/2,currentPosition.getHeading());
    		             update_VWCTRL_Parameter();
 					     exec_VWCTRL_ALGO();
    		         }
    		         else {
    	            	 temp=1;
    	            	 flag=0;
    	            	 flag0=0;
    	                 this.Drehungrichtung=2;
    		          }
    		       }
	    }
    	else if(flag==2) {
    	   if(flag0==0) {
	    	velocity=0;
	    	angularVelocity=4.0;
	    	update_VWCTRL_Parameter();
		    exec_VWCTRL_ALGO();
    	   }
	    	if (flag0==1) {
	    		 pidSetPose2.setParameter(50, 0, 0, 0);
		         if(Math.abs(currentPosition.getHeading())>0.05236/4) {
		        	 angularVelocity=pidSetPose2.getY(0,currentPosition.getHeading());
		             update_VWCTRL_Parameter();
				     exec_VWCTRL_ALGO();
		         }
		         else {
		     		   flag2=1;
		     		  this.Drehungrichtung=2;
		          }
		         }
	    } 
        break;
		case 1:
			 velocity = 50;          
			 angularVelocity=0;
			update_LINECTRL_Parameter();
           exec_LINECTRL_ALGO();
           if((navigation.getPose().getX())*100>180) flag2=2;  //position winkel um 180   
        break;
		case 2:                                 //position winkel un 180 mit 90du/3s
			 velocity = 0;          
			 angularVelocity=4.0;
		     update_VWCTRL_Parameter();
			 exec_VWCTRL_ALGO();
	        if (navigation.getPose().getHeading()>=Math.PI) flag2=3;
	     break; 		       		
		case 3:
			velocity = 50;          
			angularVelocity=0;
			update_LINECTRL_Parameter();
           exec_LINECTRL_ALGO();
           if((navigation.getPose().getX())*100>40) flag2=4;    //Position der ersten Parklueck
        break;
        case 4:
			update_PARKCTRL_Parameter();
			  exec_PARKCTRL_ALGO();
       break;		
	} 		
 }
	
	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO(){
		switch (flag5) {
		case 1:                     //hin
			 x1a = 0.0;    //Anfangsposition f黵 x1
			 x2a = 0.0;    //Anfangsposition f黵 x2
		     x1b = 0.23; //Endposition f黵 x1 20
			 x2b = -0.28; //Endposition f黵 x2 -40
			 thetai = 0.1;
			 thetaf = 0;
			 ki = 0.5;   //Anfangsgeschwindigkeit
			 kf =0.5;    //Endgeschwindigkeit
			//Parameter f黵 Polynome x1 und x2
			a0 = x1a;
			a1 = ki*Math.cos(thetai);
			a2 = 3*(x1b-x1a) - 2*ki - kf;
			a3 = 2*(x1a-x1b) + ki + kf;
			 b0 = x2a;
			 b1 =ki*Math.sin(thetai);
			 b2 = 3*(x2b-x2a);
			 b3 = 2*(x2a-x2b);
      		 T = 5; //Zeit f黵 die gesamten Fahrt 1000
	         if(t<5) {
			//Polynom f黵 s
	        //dTime=(double)PidVwControl3.getDTime()/1000;		//s
	         t=t+0.05;//0.02
			 s =(Math.pow(t,2)*(3-2*t/T))/Math.pow(T,2);
			 dots = 6*t*(1 - t/T)/Math.pow(T,2);
				//double x1 = a0 + a1*s + a2*Math.pow(s,2) + a3*Math.pow(s,3);
				double dx1 = a1 + 2*a2*s + 3*a3*Math.pow(s,2);
				double ddx1 = 2*a2 + 6*a3*s;
				//double x2 = b0 + b1*s + b2*Math.pow(s,2) + b3*Math.pow(s,3);
				double dx2 = b1 + 2*b2*s + 3*b3*Math.pow(s,2);
				double ddx2 = 2*b2 + 6*b3*s;
				velocity=200*dots*Math.sqrt(Math.pow(dx1,2) +Math.pow(dx2,2)); //Geschwindigkeit200
				angularVelocity=1.6*dots*(ddx2*dx1 - dx2*ddx1)/(Math.pow(dx1,2) +Math.pow(dx2,2)); // Winkelgeschwindigkeit
				update_VWCTRL_Parameter();
			    exec_VWCTRL_ALGO();
	         }
		      else {		    	 		   	      
		   	        	 flag5=-1;		   	            	     
			        }
	         break;
		case -1:
			 x1a = 0;    //Anfangsposition  x1
			 x2a = 0;    //Anfangsposition x2
		     x1b = 0.23; //Endposition  x1 20
			 x2b = -0.28; //Endposition x2 -40
			 thetai = 0.1;
			 thetaf = 0;
			 ki = 0.5;   //Anfangsgeschwindigkeit
			 kf =0.5;    
			a0 = x1a;
			a1 = ki*Math.cos(thetai);
			a2 = 3*(x1b-x1a) - 2*ki - kf;
			a3 = 2*(x1a-x1b) + ki + kf;
			 b0 = x2a;
			 b1 =ki*Math.sin(thetai);
			 b2 = 3*(x2b-x2a);
			 b3 = 2*(x2a-x2b);
     		 T = 5; //Zeit fuer die gesamten Fahrt 1000
	         if(t>0.1) {
	         t=t-0.05;//0.02
			 s =(Math.pow(t,2)*(3-2*t/T))/Math.pow(T,2);
			 dots = 6*t*(1 - t/T)/Math.pow(T,2);
				//double x1 = a0 + a1*s + a2*Math.pow(s,2) + a3*Math.pow(s,3);
				double dx1 = a1 + 2*a2*s + 3*a3*Math.pow(s,2);
				double ddx1 = 2*a2 + 6*a3*s;
				//double x2 = b0 + b1*s + b2*Math.pow(s,2) + b3*Math.pow(s,3);
				double dx2 = b1 + 2*b2*s + 3*b3*Math.pow(s,2);
				double ddx2 = 2*b2 + 6*b3*s;
				velocity=-200*dots*Math.sqrt(Math.pow(dx1,2) +Math.pow(dx2,2)); //Geschwindigkeit250
				angularVelocity=-1.7*dots*(ddx2*dx1 - dx2*ddx1)/(Math.pow(dx1,2) +Math.pow(dx2,2)); // Winkelgeschwindigkeit
				update_VWCTRL_Parameter();
			    exec_VWCTRL_ALGO();
	         }
		      else {
		    	  flag5=0;
			        }
	         break;
		}

		if(flag5==0) {
			      velocity=0;
	    		  angularVelocity=2.7;
	      			update_VWCTRL_Parameter();
	    		    exec_VWCTRL_ALGO();
		}
	     if (navigation.getPose().getHeading()>=0)
        {
      	 this.leftMotor.stop();
         this.rightMotor.stop(); 
         }	

		/**leftMotor.forward();
		rightMotor.forward();
		//Aufgabe 3.4
		//y(t)=P3*X(t)^3+P2*X(t)^2+P1*X(t)+P0;
		P3=27.089947089946980;
		P2=-12.190476190476131;
		P1=2.219047619047612;
		P0=0;
		setPointX=currentPosition.getX();
		setPointY=(((P3*Math.pow(setPointX, 3))+(P2*Math.pow(setPointX, 2))+(P1*setPointX))+P0); //input a,b,c,d aus Guidance
		setsetPointY1=3*P3*Math.pow(setPointX, 2)+2*P2*setPointX+P1;
		setsetPointY2=6*P3*setPointX+2*P2;
		Vges=Math.sqrt(Math.pow(setsetPointY1, 2)+1)*v0;
		omega=-v0*setsetPointY2/(1+Math.pow(setsetPointY1, 2));
			
		update_VWCTRL_Parameter();
	    exec_VWCTRL_ALGO();*/
		
	}
	
    private void exec_INACTIVE(){
    	this.stop();
	}
	
	/**
	 * DRIVING along black line
	 * Minimalbeispiel
	 * Linienverfolgung fuer gegebene Werte 0,1,2
	 * white = 0, black = 2, grey = 1
	 */
    
	private void exec_LINECTRL_ALGO(){
		leftMotor.forward();
		rightMotor.forward();
		X=(double)this.lineSensorLeftValue;
		w=(double)this.lineSensorRightValue;
		deltvauleLast=deltvaule;
		deltvaule=w-X;
		deltvauleInt=deltvauleInt+deltvaule;// intergral
		//Wtarge=0.08*deltvaule;   //nur P
		Wtarge=0.14*deltvaule+0.01*(deltvaule-deltvauleLast);  //Wtarge=0.02*deltvaule+0.00023*deltvauleInt+0.0012*(deltvaule-deltvauleLast);  //nur PI:0.08 0.0006oder 0.001(10Kreis)
		angularVelocity=Wtarge;
		update_VWCTRL_Parameter();
	    exec_VWCTRL_ALGO();

	}
 /**
     * calculates the left and right angle speed of the both motors with given velocity 
     * and angle velocity of the robot
     * @param v velocity of the robot
     * @param omega angle velocity of the robot
     */

	private void drive(double v, double omega){
		leftMotor.forward();
		rightMotor.forward();
		//Aufgabe 3.2
		dTime=(double)PidVwControl3.getDTime()/(double)1000;
		velocityLeft = distancePerDeg*(double)this.encoderLeft.getEncoderMeasurement().getAngleSum()/(double)dTime;// cm/s
		velocityRight = distancePerDeg*(double)this.encoderRight.getEncoderMeasurement().getAngleSum()/(double)dTime; // cm/s
		rightSpeed=v+(omega*trackWidth/2);
		leftSpeed=v-(omega*trackWidth/2);
		if(Drehungrichtung==0) {
		     rightSpeed = -rightSpeed ;
		     leftSpeed = -leftSpeed;
		 }

	     PidVwControl1.setParameter(0.8, 0.01, 0, 1000);//ursprunglich (0.8, 0.3, 0, 1000)
		 PidVwControl2.setParameter(0.8, 0.01, 0, 1000);
		 if(flag2==0) {
		              this.leftPower=((int)((float)PidVwControl1.getY(velocityLeft, leftSpeed)));
		              this.rightPower=((int)(1.2*(float)PidVwControl2.getY(velocityRight, rightSpeed))); 
		 }
		 else if(flag2==1){
			 this.leftPower=((int)((float)PidVwControl1.getY(velocityLeft, leftSpeed)));
			 this.rightPower=((int)((float)PidVwControl2.getY(velocityRight, rightSpeed))); 
	    }
		 if(flag5==1) {
			 this.leftPower=((int)((float)PidVwControl1.getY(velocityLeft, leftSpeed)));
			 this.rightPower=((int)(1.59*(float)PidVwControl2.getY(velocityRight, rightSpeed)));  //1.5
		 }
		 else if(flag5==-1) {
			 this.leftPower=((int)(1.1*(float)PidVwControl1.getY(velocityLeft, leftSpeed)));//1.1 
			 this.rightPower=((int)(1.6*(float)PidVwControl2.getY(velocityRight, rightSpeed)));  //1.6
			 }
		 else if(flag5==0) {
			 this.leftPower=((int)((float)PidVwControl1.getY(velocityLeft, leftSpeed)));
			 this.rightPower=((int)(1.5*(float)PidVwControl2.getY(velocityRight, rightSpeed)));  //1.5
			 }
		 leftMotor.setPower(leftPower);
		 rightMotor.setPower(rightPower);
		       //RConsole.println(leftAngSpeed  +", "+rightAngSpeed +";");
		       //RConsole.println(velocityLeft  +", "+velocityRight +";");
		 if(flag2==0) {
		               if(temp==0) {
		                    if(navigation.getPose().getX()*100>=(S0-2)) {
				                                                        flag=1;
				                                                        this.Drehungrichtung=1;
				     
				                                                      }  		    	
		                             }
		              else if(temp==1){
			                            if(navigation.getPose().getY()*100>=(S0+3)) {
				                                                                     flag=2;
				                                                                   this.Drehungrichtung=0;

		                                                                              }
		                              }
				     
		if(Drehungrichtung==1) {  
                             if (navigation.getPose().getHeading()/Math.PI*180>=90)      {
			                                                                              flag0=1;
                                                                                          }
                               }
        else if(Drehungrichtung==0) {
                                 if (navigation.getPose().getHeading()/Math.PI*180<=0){
			                                                                             flag0=1;
                                                                                       }		                 
                                                         } 
	                  }
}
	
	
	private void stop(){
		this.leftMotor.stop();
		this.rightMotor.stop();
	}
	
}