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
	//end linefollow added variable
	
	//start v/w added variable
		double dTime;
		double leftSpeed;
		double rightSpeed;
		double wheelDiameter = 5.6;
		double radius;
		double radiusL;
		double radiusR;
		double trackWidth = 15.5;
		double distancePerTurn = Math.PI*wheelDiameter;
		double distancePerDeg = distancePerTurn/360;
		double velocityLeft;
		double velocityRight;
		double velocity = 50;              //70 oder80
		double angularVelocity=0;     //2.1-Drehung um 90Â° in math. pos. Richtung;15Â°/s ;4.0-Drehung um 90Â° in math. neg. Richtung
		//double temp=0;
		//double temp1=0;
		double MVangularVelocity;
		double MVvelocity;
		int leftPower=0;
		int rightPower=0;
		double rightAngSpeed=0;
		double leftAngSpeed=0;
		//double deltS=0;
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
		  case LINE_CTRL     : update_LINECTRL_Parameter();
		                      exec_LINECTRL_ALGO();
		                      break;
		  case  VW_CTRL	:   update_VWCTRL_Parameter();
		   					  exec_VWCTRL_ALGO();
		   					  break; 
		  case SETPOSE     : update_SETPOSE_Parameter();
			  				  exec_SETPOSE_ALGO();
		                      break;
		  case PARK_CTRL 	: update_PARKCTRL_Parameter();
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

    	//Aufgabe 3.3
    	//step 1ï¼šin first quadrant ,Originposition as the starting point
    	velocity = 40; 
    	x0=currentPosition.getX()*100;            
    	y0=currentPosition.getY()*100;            
    	angle0=0;              
    	x1=destination.getX();                    
    	y1=destination.getY();                    
    	angle1=destination.getHeading();             
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
 	pidSetPose2.setParameter(8, 0, 80, 0);  //8 0 80 0
	angularVelocity=-pidSetPose2.getY(Vergleichvalue,newY);
	if (temp==1) {
		angularVelocity=-angularVelocity;
	}
  }
	
	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO(){
		//Aufgabe 3.4
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
		case -1:               //aus
			 flag5=0;
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
			    break;
	        }
	     }
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
//0.2 zu testen
		 Wtarge=0.14*deltvaule+0.01*(deltvaule-deltvauleLast);   //nur P 0.18
		//Wtarge=0.2*deltvaule+0.01*(deltvaule-deltvauleLast);//+0.005*(deltvaule-deltvauleLast)  //nur PI:0.08 0.0006oder 0.001(10Kreis)
		angularVelocity=Wtarge;
// turn Bedingung schreiben
		update_VWCTRL_Parameter();
	    exec_VWCTRL_ALGO();
/**		pidLineFollow1.setParameter(0.2, 0.0026, 0.13,1000);
		controlOut = pidLineFollow1.getY(X,w);	
		rightMotor.setPower((int)(highPower+controlOut));
		leftMotor.setPower((int)(highPower-controlOut));
		*/
	}
 /**
     * calculates the left and right angle speed of the both motors with given velocity 
     * and angle velocity of the robot
     * @param v velocity of the robot
     * @param omega angle velocity of the robot
     */

	private void drive(double v, double omega){
		//Aufgabe 3.2
		dTime=(double)PidVwControl3.getDTime()/(double)1000;
		velocityLeft = distancePerDeg*(double)this.encoderLeft.getEncoderMeasurement().getAngleSum()/(double)dTime;// cm/s
		velocityRight = distancePerDeg*(double)this.encoderRight.getEncoderMeasurement().getAngleSum()/(double)dTime; //  cm/s
		rightSpeed=v+(omega*trackWidth/2);
		leftSpeed=v-(omega*trackWidth/2);
		if(Drehungrichtung==0) {
		     rightSpeed = -rightSpeed ;
		     leftSpeed = -leftSpeed;
		 }
		 double deltS=velocityRight*dTime;
         S=S+deltS;
// pid-parameter zu testen
	     PidVwControl1.setParameter(0.8, 0.01, 0, 1000);//ursprunglich 0.8
		 PidVwControl2.setParameter(0.8, 0.01, 0, 1000);
//mal Koeffiezient?
		 if(flag5==1) {
			 this.leftPower=((int)((float)PidVwControl1.getY(velocityLeft, leftSpeed)));
			 this.rightPower=((int)(1.59*(float)PidVwControl2.getY(velocityRight, rightSpeed)));  //1.5
		 }
		 else if(flag5==-1) {
			 this.leftPower=((int)(1.1*(float)PidVwControl1.getY(velocityLeft, leftSpeed)));//1.1 
			 this.rightPower=((int)(1.6*(float)PidVwControl2.getY(velocityRight, rightSpeed)));  //1.6
			 }
		 else if(flag5==0) {
			 this.leftPower=(int) (Math.round((float)PidVwControl1.getY(velocityLeft, leftSpeed)));
			 this.rightPower=(int) (Math.round((float)PidVwControl2.getY(velocityRight, rightSpeed)));
			 }

		 leftMotor.setPower(leftPower);
		 rightMotor.setPower(rightPower);
		       //RConsole.println(leftAngSpeed  +", "+rightAngSpeed +";");
		       //RConsole.println(velocityLeft  +", "+velocityRight +";");	   
		 /**		 if(S>=10) {
		    	flag=2;
		    	this.Drehungrichtung=1;
		    } 
		 
		 
		 if(Drehungrichtung==1) {  
             if (navigation.getPose().getHeading()/Math.PI*180>=0)    //(S>=(Math.PI*trackWidth/2-1.2))  //Drehung um 90Â° in math. pos. Richtung, 15Â°/s  
               {
            	 this.leftMotor.stop();
                 this.rightMotor.stop(); 
                }
           }
          else if(Drehungrichtung==0) {
             if (navigation.getPose().getHeading()/Math.PI*180<=(-88))//(S<=(6.8-Math.PI*trackWidth/4))  //Drehung um 90Â° in math. neg. Richtung, 30Â°/s  
              {
                flag=1;
                this.Drehungrichtung=2;
               }		                 
          }  */
}
	
	
	private void stop(){
		this.leftMotor.stop();
		this.rightMotor.stop();
	}
	
}