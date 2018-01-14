package parkingRobot.hsamr4;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
import lejos.robotics.navigation.Pose;

import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation;
import parkingRobot.INavigation.ParkingSlot;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import parkingRobot.hsamr4.ControlRST;
import parkingRobot.hsamr4.HmiPLT;
import parkingRobot.hsamr4.NavigationAT;
import parkingRobot.hsamr4.PerceptionPMP;

import lejos.geom.Line;
import lejos.nxt.LCD;



/**
 * Main class for 'Hauptseminar AMR' project 'autonomous parking' for students of electrical engineering
 * with specialization 'automation, measurement and control'.
 * <p>
 * Task of the robotic project is to develop an mobile robot based on the Lego NXT system witch can perform
 * parking maneuvers on an predefined course. To fulfill the interdisciplinary aspect of this project the software
 * structure is divided in 5 parts: human machine interface, guidance, control, perception and navigation.
 * <p>
 * Guidance is to be realized in this main class. The course of actions is to be controlled by one or more finite
 * state machines (FSM). It may be advantageous to nest more than one FSM.
 * <p>
 * For the other parts there are interfaces defined and every part has to be realized in one main module class.
 * Every class (except guidance) has additionally to start its own thread for background computation.
 * <p>
 * It is important that data witch is accessed by more than one main module class thread is only handled in a
 * synchronized context to avoid inconsistent or corrupt data!
 */
public class GuidanceAT {
	
	/**
	 * states for the main finite state machine. This main states are requirements because they invoke different
	 * display modes in the human machine interface.
	 */
	public enum CurrentStatus {
		/**
		 * indicates that robot is following the line and maybe detecting parking slots
		 */
		DRIVING,
		/**
		 * indicates that robot is parking in selected parking slot
		 */
		PARK_THIS,
		/**
		 * indicates that robot is performing an parking maneuver
		 */
		INACTIVE,
		/**
		 * indicates that shutdown of main program has initiated
		 */
		EXIT
	}
	
	/**
	 * Zustandsautomat f��r DRIVING
	 * @author lenovo
	 *
	 */
	private enum SubMissionDriving{  
		
	   
		/**
		 * schauen ob ein slot existiert
		 */
		SUCHEN,	
		
		/**
		 * Slot langsam vorbei fahren
		 */
		PASS_BY,
		
		
		
	}	
	
	/**
	 * Zustandsautomat f��r ParkThis
	 * @author lenovo
	 *
	 */
     private enum SubMissionParkThis {
		/**
		 * drive to slot
		 */
		DRIVE_TO,
		/**
		 * parking into slot
		 */		
		PARK_IN,
		/**
		 * solange drehen bis parallel zur Seitenwand in der Parkl��cke stehen
		 */
		WINKEL_ADJUSTMENT,
		/**
		 *vorwaerts oder rueckwaerts fahren bis am Ende mittiger in der Parkl��cke eingeparkt
		 */
		POSITIONSKORREKTUR,
	}
     /**
 	 * aktueller Zustand von Driving
 	 */	
 	protected static SubMissionDriving missionDriving 	= SubMissionDriving.SUCHEN;
    /**
     * aktueller Zustand von Parkthis 
     */
    protected static SubMissionParkThis missionThis 		= SubMissionParkThis.DRIVE_TO;
	/**
	 * state in which the main finite state machine is running at the moment
	 */
	protected static CurrentStatus currentStatus 	= CurrentStatus.INACTIVE;
	/**
	 * state in which the main finite state machine was running before entering the actual state
	 */
	protected static CurrentStatus lastStatus		= CurrentStatus.INACTIVE;
	
	
	/**
	 * one line of the map of the robot course. The course consists of a closed chain of straight lines.
	 * Thus every next line starts where the last line ends and the last line ends where the first line starts.
	 * This documentation for line0 hold for all lines.
	 */
	static Line line0 = new Line(  0,  0, 180,  0);
	static Line line1 = new Line(180,  0, 180, 60);
	static Line line2 = new Line(180, 60, 150, 60);
	static Line line3 = new Line(150, 60, 150, 30);
	static Line line4 = new Line(150, 30,  30, 30);
	static Line line5 = new Line( 30, 30,  30, 60);
	static Line line6 = new Line( 30, 60,   0, 60);
	static Line line7 = new Line(  0, 60,   0,  0);
	/**
	 * map of the robot course. The course consists of a closed chain of straight lines.
	 * Thus every next line starts where the last line ends and the last line ends where the first line starts.
	 * All above defined lines are bundled in this array and to form the course map.
	 */
	static Line[] map = {line0, line1, line2, line3, line4, line5, line6, line7};
	
	
	/**
	 * main method of project 'ParkingRobot'
	 * 
	 * @param args standard string arguments for main method
	 * @throws Exception exception for thread management
	 */
	public static void main(String[] args) throws Exception {		
       
	
		
		// Generate objects
		
		NXTMotor leftMotor  = new NXTMotor(MotorPort.B);
		NXTMotor rightMotor = new NXTMotor(MotorPort.A);
		
		IMonitor monitor = new Monitor();
		
		IPerception perception = new PerceptionPMP(leftMotor, rightMotor, monitor);
		perception.calibrateLineSensors();
		
		INavigation navigation = new NavigationAT(perception, monitor);
		IControl    control    = new ControlRST(perception, navigation, leftMotor, rightMotor, monitor);
		INxtHmi  	hmi        = new HmiPLT(perception, navigation, control, monitor);
		
		monitor.startLogging();
		
	//Zustand initialization
		
		currentStatus = CurrentStatus.INACTIVE;
        lastStatus    = CurrentStatus.EXIT;
        
    //Variableninitialisierung
		
        double etaAbstand = 0.05;						//Toleranzparameter
        double etaWinkel = 5;
	    ParkingSlot[] slotList = navigation.getParkingSlots();			//Liste der Parkl��ken
		ParkingSlot slot = null;				    //Aktuelle Parkl��ke
		Pose pose = null;						//Positionsvariable
		boolean parkedin = false;					//Hilfsvariable f��r ausparken
		navigation.setDetectionState(false);		//Parkl��ken detektieren
		boolean stateJudge = true;	           //Zustand��bergang pr��fen
		
	
	    
				
		while(true) {
			showData(navigation, perception);				//Displayausgabe						
			pose = navigation.getPose();					//Pose aktualisieren
			slotList = navigation.getParkingSlots();

///////////////////////main states/////////////////////////////
			
        	switch ( currentStatus )
        	{
        	
////////////////////////DRIVING////////////////////////////////
        	
				case DRIVING:
					// MONITOR (example)
//					monitor.writeGuidanceComment("Guidance_Driving");
					
					//Into action
					if ( lastStatus != CurrentStatus.DRIVING )
					{
						if(parkedin==true) 
						{
							stateJudge=false;
							control.setParkingDirection(-1);			                //Parkrichtung auf ausparken stellen
							control.setCtrlMode(ControlMode.PARK_CTRL);	            // PARK_CTRL aktivieren
								
							//Ausparkvorgang beendet?                                                
							if ((slot.getFrontBoundaryPosition().getX()+etaAbstand>pose.getX())&&(slot.getFrontBoundaryPosition().getX()-etaAbstand<pose.getX())&&
									(slot.getFrontBoundaryPosition().getY()+etaAbstand>pose.getY())&&(slot.getFrontBoundaryPosition().getY()-etaAbstand<pose.getY())) 
							{	
							slot=null;
							parkedin=false;
							stateJudge=true;
							}
						}                                                       
						
						else 
						{
							slot=null;
							navigation.setDetectionState(true);
							control.setCtrlMode(ControlMode.LINE_CTRL); // following the line
						}
						
				        missionDriving = SubMissionDriving.SUCHEN; //Driving Zustandsautomaten initialisieren
					    
						}
					
					//While action	
					{	
					
						/////////////Driving Zustandsautomat////////////
						switch (missionDriving ) {
							
						case SUCHEN:
							slot=getGoodSlot(slotList, pose, etaAbstand);  //Signalisiert bei gefundenem Slot
							
							if(slot!=null)                        //pr��fen ob slot gefunden wurde
							{
							
							Sound.buzz();
								//Sound.twoBeeps();
							//setPose auf endpunkt der Parklueke
							control.setDestination(navigation.getPose().getHeading(), slot.getFrontBoundaryPosition().getX(), slot.getFrontBoundaryPosition().getY());
							control.setVelocity(20);
							control.setCtrlMode(ControlMode.SETPOSE);
						
							missionDriving = SubMissionDriving.PASS_BY;			//Zustandsuebergang
							
							}
							
							break;
							
						
						case PASS_BY:
							
							//Abfrage ob Endposition des Slots erreicht ist
							if ((slot.getFrontBoundaryPosition().getY()+etaAbstand>pose.getY())&&(slot.getFrontBoundaryPosition().getY()-etaAbstand<pose.getY())&&
									(slot.getFrontBoundaryPosition().getX()+etaAbstand>pose.getX())&&(slot.getFrontBoundaryPosition().getX()-etaAbstand<pose.getX()))
								{
								control.setCtrlMode(ControlMode.LINE_CTRL);			//LINE_CTRL wieder aktivieren
								slot=null;	//slot resetten
							
								missionDriving = SubMissionDriving.SUCHEN ;			//Zustands黚ergang
								}
							break;
						default:
							break;

						
						}
					}				
					
					//State transition check
					lastStatus = currentStatus;
					if(stateJudge) 
					{
						if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS)
						{
						    currentStatus = CurrentStatus.PARK_THIS;
						}
					}
					if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE )
					{
						currentStatus = CurrentStatus.INACTIVE;
					}
					else if ( Button.ENTER.isDown() )
					{
						currentStatus = CurrentStatus.INACTIVE;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}
					else if ( Button.ESCAPE.isDown() )
					{
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}
					else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT)
					{
						currentStatus = CurrentStatus.EXIT;
					}
					
					//Leave action
					if ( currentStatus != CurrentStatus.DRIVING )
					{
						//nothing to do here
					}
					break;	
					
////////////////////////////////////Park_This////////////////////////////////////
                   case PARK_THIS:
					
					//Into action
					if ( lastStatus != CurrentStatus.PARK_THIS )						
					{
						slot = getParkingSlot(hmi.getSelectedParkingSlot(), slotList); 	//Slot anhand der ID aus der Liste holen
						if (slot!=null)
						{
							Sound.beep();												//Signalisiert erfolgreiche Slotentgegennahme
						}
						control.setCtrlMode(ControlMode.LINE_CTRL);						//LINE_CTRL aktivieren
						missionThis = SubMissionParkThis.DRIVE_TO;							//ParkThis Zusatndautomat initialisieren
						
					}
					
					// While action
					{
						//Park_This Zustandautomat
						switch (missionThis) {
						case DRIVE_TO:
							//Prueft ob Anfangsposition erreicht ist
							if ((slot.getBackBoundaryPosition().getY()+etaAbstand>pose.getY())&&(slot.getBackBoundaryPosition().getY()-etaAbstand<pose.getY())&&
									(slot.getBackBoundaryPosition().getX()+etaAbstand>pose.getX())&&(slot.getBackBoundaryPosition().getX()-etaAbstand<pose.getX())) 
							{
								stateJudge=false;
								navigation.setDetectionState(false);		//Deaktiviert Detection
								
								//prueft ob der Roboter am Anfangsposition paralle auf schwarz linie 
								if ( slot.getID()==1||slot.getID()==2)
								{
								 while (pose.getHeading()/Math.PI*180>0)
								   {
									   control.setVelocity(0);
									   control.setAngularVelocity(-2.1);
									   control.setCtrlMode(ControlMode.VW_CTRL);
								   }
								 //wenn Phi<0, dreht der Roboter nach links bis phi gleich 0
								   while (pose.getHeading()/Math.PI*180<0)
								   {
									   control.setVelocity(0);
									   control.setAngularVelocity(2.1);
									   control.setCtrlMode(ControlMode.VW_CTRL);
								   } 
								   if ((pose.getHeading()/Math.PI*180>0-etaWinkel)&&(pose.getHeading()/Math.PI*180>0+etaWinkel))
								   {
									   missionThis = SubMissionParkThis.PARK_IN;	//Zustandsuebrgang
								   }
								}
								if (slot.getID()==3)
								{
									while (pose.getHeading()/Math.PI*180>90)
									   {
										   control.setVelocity(0);
										   control.setAngularVelocity(-2.1);
										   control.setCtrlMode(ControlMode.VW_CTRL);
									   }
									 //wenn Phi<0, dreht der Roboter nach links bis phi gleich 0
									   while (pose.getHeading()/Math.PI*180<90)
									   {
										   control.setVelocity(0);
										   control.setAngularVelocity(2.1);
										   control.setCtrlMode(ControlMode.VW_CTRL);
									   } 
									   if ((pose.getHeading()/Math.PI*180>90-etaWinkel)&&(pose.getHeading()/Math.PI*180>90+etaWinkel))
									   {
										   missionThis = SubMissionParkThis.PARK_IN;	//Zustandsuebrgang
									   }
								}
								
								if (slot.getID()==4)
								{
									while (pose.getHeading()/Math.PI*180>180)
									   {
										   control.setVelocity(0);
										   control.setAngularVelocity(-2.1);
										   control.setCtrlMode(ControlMode.VW_CTRL);
									   }
									 //wenn Phi<0, dreht der Roboter nach links bis phi gleich 0
									   while (pose.getHeading()/Math.PI*180<180)
									   {
										   control.setVelocity(0);
										   control.setAngularVelocity(2.1);
										   control.setCtrlMode(ControlMode.VW_CTRL);
									   } 
									   if ((pose.getHeading()/Math.PI*180>180-etaWinkel)&&(pose.getHeading()/Math.PI*180>180+etaWinkel))
									   {
										   missionThis = SubMissionParkThis.PARK_IN;	//Zustandsuebrgang
									   }
								}
							}
							break;
							
						case PARK_IN:
					
    							control.setParkingDirection(1);				//Setzte Parkrichtung
								control.setCtrlMode(ControlMode.PARK_CTRL);		//Setzt PARK_CTRL
								
					           //Einparkenvorgang beendet?
                               //3 Situationen: 1. Slot auf line 0; 2. Slot auf line 1; 3.Slot auf line  4								
								
								if(((pose.getX()<slot.getBackBoundaryPosition().getX()+0.23+etaAbstand)&&(pose.getX()>slot.getBackBoundaryPosition().getX()+0.23-etaAbstand)&&
										(pose.getY()<slot.getBackBoundaryPosition().getY()-0.29+etaAbstand)&&(pose.getY()>slot.getBackBoundaryPosition().getY()-0.29-etaAbstand))
										||((pose.getX()<slot.getBackBoundaryPosition().getX()+0.29+etaAbstand)&&(pose.getX()>slot.getBackBoundaryPosition().getX()+0.29-etaAbstand)&&
												(pose.getY()<slot.getBackBoundaryPosition().getY()+0.23+etaAbstand)&&(pose.getY()>slot.getBackBoundaryPosition().getY()+0.23-etaAbstand))
										||((pose.getX()<slot.getBackBoundaryPosition().getX()-0.23+etaAbstand)&&(pose.getX()>slot.getBackBoundaryPosition().getX()-0.23-etaAbstand)&&
												(pose.getY()<slot.getBackBoundaryPosition().getY()+0.29+etaAbstand)&&(pose.getY()>slot.getBackBoundaryPosition().getY()+0.29-etaAbstand)))
								{   missionThis = SubMissionParkThis.WINKEL_ADJUSTMENT;
								}
								break;
								
						case WINKEL_ADJUSTMENT:
							
							if ( slot.getID()==1||slot.getID()==2)
							{
							 while (pose.getHeading()/Math.PI*180>0)
							   {
								   control.setVelocity(0);
								   control.setAngularVelocity(-2.1);
								   control.setCtrlMode(ControlMode.VW_CTRL);
							   }
							   while (pose.getHeading()/Math.PI*180<0)
							   {
								   control.setVelocity(0);
								   control.setAngularVelocity(2.1);
								   control.setCtrlMode(ControlMode.VW_CTRL);
							   } 
							   if ((pose.getHeading()/Math.PI*180>0-etaWinkel)&&(pose.getHeading()/Math.PI*180>0+etaWinkel))
							   {
								   missionThis = SubMissionParkThis.PARK_IN;	//Zustandsuebrgang
							   }
							}
							if (slot.getID()==3)
							{
								while (pose.getHeading()/Math.PI*180>90)
								   {
									   control.setVelocity(0);
									   control.setAngularVelocity(-2.1);
									   control.setCtrlMode(ControlMode.VW_CTRL);
								   }
								   while (pose.getHeading()/Math.PI*180<90)
								   {
									   control.setVelocity(0);
									   control.setAngularVelocity(2.1);
									   control.setCtrlMode(ControlMode.VW_CTRL);
								   } 
								   if ((pose.getHeading()/Math.PI*180>90-etaWinkel)&&(pose.getHeading()/Math.PI*180>90+etaWinkel))
								   {
									   missionThis = SubMissionParkThis.PARK_IN;	//Zustandsuebrgang
								   }
							}
							
							if (slot.getID()==4)
							{
								while (pose.getHeading()/Math.PI*180>180)
								   {
									   control.setVelocity(0);
									   control.setAngularVelocity(-2.1);
									   control.setCtrlMode(ControlMode.VW_CTRL);
								   }
								   while (pose.getHeading()/Math.PI*180<180)
								   {
									   control.setVelocity(0);
									   control.setAngularVelocity(2.1);
									   control.setCtrlMode(ControlMode.VW_CTRL);
								   } 
								   if ((pose.getHeading()/Math.PI*180>180-etaWinkel)&&(pose.getHeading()/Math.PI*180>180+etaWinkel))
								   {
									   missionThis = SubMissionParkThis.PARK_IN;	//Zustandsuebrgang
								   }
							}
							     break;
							    
					    
						case POSITIONSKORREKTUR:
							
						 	//wenn der Abstand nach vorne groesser als der Abstand nach hinten, vorwaertsfahren
							while (perception.getFrontSensorDistance()>perception.getBackSensorDistance())
							{		
								       control.setVelocity(12);
								       control.setAngularVelocity(0);
								       control.setCtrlMode(ControlMode.VW_CTRL);
							}
									
							// wenn der Abstand nach vorne kleiner als der Abstand nach hinten, r��ckwaertsfahren
									
							while (perception.getFrontSensorDistance()<perception.getBackSensorDistance())
							{		
								       control.setVelocity(-12);
								       control.setAngularVelocity(0);
								       control.setCtrlMode(ControlMode.VW_CTRL);
							}	
							
							if( (perception.getFrontSensorDistance()-perception.getBackSensorDistance()>=-0.03)&&
									(perception.getFrontSensorDistance()-perception.getBackSensorDistance()<=0.03))
							{
							    parkedin=true;	
							    stateJudge=true;
								control.setCtrlMode(ControlMode.INACTIVE);		//Pausiert Control
								currentStatus = CurrentStatus.INACTIVE;		//Zustandsuebrgan
							}	
							break;

						default:
							break;
						}
						
						//State transition check
						lastStatus = currentStatus;
						if (stateJudge)
						{
						    if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT )
						    {
							currentStatus = CurrentStatus.DRIVING;						
						    }
						    else if ( Button.ENTER.isDown() )
						    {
							currentStatus = CurrentStatus.INACTIVE;
							while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
						    }
						    else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE )
						    {
							currentStatus = CurrentStatus.INACTIVE;
							}
						   
						}
						
			            if ( Button.ESCAPE.isDown() )
						{
							currentStatus = CurrentStatus.EXIT;
							while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
						}
						else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT)
						{
							currentStatus = CurrentStatus.EXIT;
						}		
						
						//Leave action
						if ( currentStatus != CurrentStatus.PARK_THIS ){
							
						}					
						break;
					}
////////////////////////////////////INACTIVE/////////////////////////////////////
					
					case INACTIVE:
				
					//Into action
					if ( lastStatus != CurrentStatus.INACTIVE ){
						control.setCtrlMode(ControlMode.INACTIVE);
					}
					
					//While action
					{
						//nothing to do here
					}
					
					//State transition check
					lastStatus = currentStatus;
					if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT )
					{
						currentStatus = CurrentStatus.DRIVING;						
					}
					else if ( Button.ENTER.isDown() )
					{
						currentStatus = CurrentStatus.DRIVING;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}
					else if ( Button.ESCAPE.isDown() )
					{
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}
					else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT)
					{
						currentStatus = CurrentStatus.EXIT;
					}
					else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS)
					{
					    currentStatus = CurrentStatus.PARK_THIS;
					}			
					
					//Leave action
					if ( currentStatus != CurrentStatus.INACTIVE ){
						//nothing to do here
					}					
					break;
					
////////////////////////////////EXIT///////////////////////////////////
					
					case EXIT:
					hmi.disconnect();
					/** NOTE: RESERVED FOR FUTURE DEVELOPMENT (PLEASE DO NOT CHANGE)
					// monitor.sendOfflineLog();
					*/
					monitor.stopLogging();
					System.exit(0);
					break;
			default:
				break;
        	}
        		
        	Thread.sleep(100);        	
		}
	}
	
	
/**
 *  Die Methode prueft ob eine Parklueke in der Liste existiert, die den Status suitable ist und deren Anfangspunkt in der Pose ist.
 * @param slotList enthaelt die zu pruefende Parklueke
 * @param pose zu pruefende Position
 * @param a Toleranzvariable
 * @return "null" falls keine solche Parklueke existiert bzw. die gefundene Parklueke
 */
	private static ParkingSlot getGoodSlot(ParkingSlot[] slotList, Pose pose, Double a) 
	
	{
		if(slotList ==  null)
		{
			return null;								//Die ParkingSlotliste ist Leer
		}
		else
		{
			 
			for(int i = 0; i < slotList.length; i++)
			{
				//Pfr��ft ob eine Parkl��ke mit den Anforderungen existiert
				if ((slotList[i].getStatus()==ParkingSlotStatus.SUITABLE_FOR_PARKING)&&(slotList[i].getBackBoundaryPosition().getY()+a>pose.getY())&&(slotList[i].getBackBoundaryPosition().getY()-a<pose.getY())&&
						(slotList[i].getBackBoundaryPosition().getX()+a>pose.getX())&&(slotList[i].getBackBoundaryPosition().getX()-a<pose.getX()))
				{
					return slotList[i];	//Gibt diese Parkl��ke zur��ck
				}
			}
			
			return null; 				//Es gibt keine solche L��ke
		}
	}	

	
	/**
	 * Durchsucht eine Liste mit Parklueken nach einer Parklueke mit der gewaehlte ID
	 * @param selectedParkingSlot gesuchte Parklueke
	 * @param slotList Parkluekenlist
	 * @return "null" falls keine solche Parklueke existiert bzw. die gefundene Parklueke
	 */
	private static ParkingSlot getParkingSlot(int selectedParkingSlot, ParkingSlot[] slotList) 
	{
		
			if(slotList ==  null) 
			{
				return null;						//Die ParkingSlotliste ist Leer
			}
			else
			{
				
				for(int i = 0; i <slotList.length; i++)
				{
					if(slotList[i].getID() == selectedParkingSlot)		
						return slotList[i];			//Gibt die gesuchte Parklueke zuruek
				}
				
				return null;						//Die ID ist nicht vorhanden
			}
		}	
	


	/**
	 * returns the actual state of the main finite state machine as defined by the requirements
	 * 
	 * @return actual state of the main finite state machine
	 */
	public static CurrentStatus getCurrentStatus(){
		return GuidanceAT.currentStatus;
	}
	
	/**
	 * plots the actual pose on the robots display
	 * 
	 * @param navigation reference to the navigation class for getting pose information
	 */
	protected static void showData(INavigation navigation, IPerception perception){
		LCD.clear();	
	
		LCD.drawString("X (in cm): " + (navigation.getPose().getX()*100), 0, 0);
		LCD.drawString("Y (in cm): " + (navigation.getPose().getY()*100), 0, 1);
		LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading()/Math.PI*180), 0, 2);
		LCD.drawString("FS:"+Integer.toString((int)perception.getFrontSideSensorDistance()), 0, 3);
	
	}
}