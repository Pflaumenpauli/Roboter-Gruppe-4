package parkingRobot.hsamr0;

public class PidC {
	double Kp=0;
	double Ki=0;
	double Kd=0;
	double windUp=0;
	double X=0;
	//Führungsgröße
	double w=0;
	//Regeldifferenz
	double e=0;
	//Regeldifferenz beim vorherigen durchlauf des Threads
    double eLast=0;
	double eInt=0;
	 //Reglerausgangsgröße
    double y=0;
    long  dTime=0;
    long time=System.currentTimeMillis();
    
    public void setParameter(double Kp,double Ki, double Kd, double windUp){
		this.Kp=Kp;
		this.Ki=Ki;
		this.Kd=Kd;
		this.windUp=windUp;
    }
    
    
    public double getY(double X, double w)
    {
    	this.X=X;
    	this.w=w;
    	eLast=e;
    	//7 ist Lichtvaluedifferenz zwischen Zwei Lichtsensor
    	e=w-X;
    	eInt=eInt+e;
    	if(eInt>windUp){
			eInt=(double)windUp;
		}
		else if(eInt<(-windUp)){
			eInt=(double)(-windUp);
		}
		y=Kp*e+Ki*eInt+Kd*(e-eLast);
    	
		return y ;
    	
    }
    public long getDTime(){
		dTime=System.currentTimeMillis()-time;
		time=System.currentTimeMillis();
		return dTime;
	}
	
}
