

import lejos.nxt.*;
import lejos.nxt.addon.*;
//import lejos.util.KalmanFilter;
import lejos.util.Integration;

public class NXTRobotDraw{
    /**
     * @param args
     */
	
	//Side Sizes
	public static final int side=900;
	public static final int side2=1800;
	
	//Instancing Attributes
	public static CompassHTSensor comp=new CompassHTSensor(SensorPort.S2);
    public static AccelHTSensor accel=new AccelHTSensor(SensorPort.S4);
    
    // Filter Auxiliar
    public static double last=0.0;

    //Amazing Class for Integrations
    //speed to position
    public static Integration Ipos;
    //acceleration to speed
    public static Integration Ivel;

    
    
    
    
    //Filtering Degree
	public static double getDegreeCalc(){
		
		//
		double tau=0.5,degree;
        
        double aux=0.0,e=0.0;		
		//Filter
    	aux=comp.getDegreesCartesian();
    	e=(last-aux)*tau;
    	last=aux;
    	degree=e+aux;
    	
    	return degree;
    	
	}
	
	
	
	//Calculating 
	public static double CalculatePosition(Integration Iv,Integration Ip) {
	
	    
		double result;	
//		result=(double)accel.getZAccel();
		//Obtaining Acceleration Value Axis X
		result=(double)accel.getXAccel();
		//Integration for obtaining speed
		result=Iv.addReading(result);
		//Integrating again for obtaining position
		result=Ip.addReading(result);
		
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
        
        double posChange;
        double initialDegree;
        double finalDegree;
        double accumPos;
        boolean flag=true;
        
        //Starting Compass
        comp.startCalibration();
        Thread.sleep(3000);
        comp.stopCalibration();
        
        //Reset Cartesian Degrees
        comp.resetCartesianZero();
     
        //Setting Speed
        Motor.A.setSpeed(30);
        Motor.C.setSpeed(30);
             
        
        //while (!touch.isPressed())
        for(int i=0;i<12;i++)
        {
//KALMAN
//        	x=comp.getDegreesCartesian();
//        	p=p+q;
//        	k=p/(p+r);
//        	degree=x+k*(degree-x);
//        	p=(1-k)*p;
        	
        	
            
            //speed to position
            Ipos=new Integration(0,0);
            //acceleration to speed
            Ivel=new Integration(0,0);

        	
        	//Calculating Position from Accelerometer        
        	posChange=CalculatePosition(Ivel,Ipos);
        	       	
        	//Setting final Position
        	if(i%2==0) //long Side
        	accumPos=posChange+side2;
        	else //ShortSide
        	accumPos=posChange+side;
        	
        	
        	
        	//Displaying Values
        	System.out.print ("Current Pos."+posChange+"\n");
        	System.out.print("Final Pos. "+accumPos+"\n");
        	
        	//Stop without Inertia
        	Motor.A.stop(true);
        	Motor.C.stop(true);
        	
        	//Wait to See
        	Thread.sleep(1000);
        	
        	//Rotating Robot
        	Motor.A.forward();
            Motor.C.forward();
        	
            
            //Going to front
            while (posChange<accumPos)
        	posChange=CalculatePosition(Ivel,Ipos);

            
            //Stop without Inertia
        	Motor.A.stop(true);
        	Motor.C.stop(true);
        	
        	
        	// Calculating Degrees
        	initialDegree=getDegreeCalc();        	
        	finalDegree=initialDegree+90.0;
        	
        	
        	//If InitialDegree is more than 270°
        	if (finalDegree>360)
        		finalDegree=finalDegree-360;     	
        	        	
         	System.out.print(initialDegree+" I.Deg. \n");
        	System.out.print(finalDegree+" F.Deg. \n");

        	//Wait to See Screen
        	Thread.sleep(1000);
           	
        	//Starts Rotating
        	Motor.A.forward();
            Motor.C.backward();
            
           
            
        	do
        	{               
        	initialDegree=getDegreeCalc();
        	System.out.print(initialDegree+"\n");

        	//When Movement doesn't pass by 0°
        	if (finalDegree>=90)
        		if (initialDegree>=finalDegree )
        			flag=false;
        							
        	//When Movement pass by 0°
        	if (finalDegree<90)
        		if (finalDegree>=initialDegree)
        			flag=false;
        	}
        	while (flag);

        	 //Reset Rotating
            flag=true;
        	
        }
 
    
    
    
    
    
    
    
    
    
    
    }
}