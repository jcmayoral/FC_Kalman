

import lejos.nxt.*;
import lejos.nxt.addon.*;
//import lejos.util.KalmanFilter;
import lejos.util.Integration;

public class NXTRobotDraw{
    /**
     * @param args
     */
	
	
	//Instancing Attributes
	public static CompassHTSensor comp=new CompassHTSensor(SensorPort.S2);
    public static AccelHTSensor accel=new AccelHTSensor(SensorPort.S4);
    
    
    //Amazing Class for Integrations
    //speed to position
    public static Integration Ipos=new Integration(0,0);
    //acceleration to speed
    public static Integration Ivel=new Integration(0,0);
    
    
    
    
    //Filtering Degree
	public static double getDegreeCalc (double degree){
		
		double tau=0.3;
        double aux=0.0;
        double e=0.0,last=0.0;		
		//Filter
    	aux=comp.getDegreesCartesian();
    	last=aux;
    	e=(last-aux)*tau+e;
    	degree=e+aux;
    	return degree;
	}
	
	
	//Calculating 
	public static double CalculatePosition() {
		
		double result;	
		result=(double)accel.getZAccel();
		result=Ivel.addReading(result);
		result=Ipos.addReading(result);
		
		return result;
		
	}
	
	

	
	
	
    public static void main(String[] args) throws Exception {
    	       
//    	double[][] H={{1,0},{0,1}}; //Sensor Model we don't have it so we are skipping
//    	double[][] Pk={{1,0},{0,1}}; //Covariance Matrix is eye Matrix because we only have one dimension sensor
//    	double[][] Fk={{1,0},{0,1}}; //Prediction State Matrix
//        double[][] degree={{0},{0}};  //Degree Arraay "X"
//        
/*
 * Update Pk = Fk*Pk(k-1)*fkT+Qk (Qk=process noise)
 * xk=
 * 
 * Hk=xk
 */
       
        /*
 * x=x
 * p=p+q
 * k=p/(p+r)
 * x=x+k*(measurement-x)
 * p=(1-k)*p
 * 
 * x filtered value
 * q process noise 
 * r sensor node
 * p estimated error
 * k Kalmann Gain
 * 
 * start with with q, r, p and x
 */
        
        //double q=0.2,r=0.1,p=1,k=1,x; Kalman
        double posChange=0;
        double initialDegree;
        double finalDegree;
        double accumPos;
        
        
        //Starting Compass
        comp.startCalibration();
        Thread.sleep(3000);
        comp.stopCalibration();
        comp.resetCartesianZero();
     
        //Setting Speed
        Motor.A.setSpeed(20);
        Motor.C.setSpeed(20);
             
        
        //while (!touch.isPressed())
        for(int i=0;i<1;i++)
        {
//KALMAN
//        	x=comp.getDegreesCartesian();
//        	p=p+q;
//        	k=p/(p+r);
//        	degree=x+k*(degree-x);
//        	p=(1-k)*p;

        	
        	//Complementary
        
        	posChange=CalculatePosition();
        	accumPos=posChange+1000;
        	
        	do
        	{
        	posChange=CalculatePosition();
        	Motor.A.forward();
            Motor.C.forward();
            	
        	}
        	while (posChange<accumPos);       	
        	
        	initialDegree=comp.getDegreesCartesian();
        	System.out.print(initialDegree+"initial \n");
        	Thread.sleep(3000);
        	
        	finalDegree=initialDegree+90.0;
        	
        	if (finalDegree>360)
        		finalDegree=finalDegree-360;     	
        	
        	System.out.print(finalDegree+"final \n");
        	Thread.sleep(3000);
        	
        	Motor.A.forward();
            Motor.C.backward();        	
        	do
        	{               
        	initialDegree=getDegreeCalc(initialDegree);
        	System.out.print(initialDegree+"\n");
        	}
        	while (initialDegree<finalDegree);

        }
 
    
    
    
    
    
    
    
    
    
    
    }
}