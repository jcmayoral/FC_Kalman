

import lejos.nxt.*;
import lejos.nxt.addon.*;
//import lejos.util.KalmanFilter;
import lejos.util.Integration;

public class NXTRobotDraw{
    /**
     * @param args
     */
	
	//Side Sizes
	public static final double side=10;
	public static final double side2=20;
	
	//Instancing Attributes
	public static CompassHTSensor comp=new CompassHTSensor(SensorPort.S2);
    public static AccelHTSensor accel=new AccelHTSensor(SensorPort.S4);
    
   // Filter Auxiliar
    public static double last=0.0;
    public static double tau=0.4;
    
    //Amazing Class for Integrations
    //speed to position
    public static Integration Ipos;
    //acceleration to speed
    public static Integration Ivel;

    
    public static void robotRotation()
    {
       	double dRadius=4,distance,rotations,degreescount;
    	double pi=3.1416;
    	
    	//perimeter of complete Circle 
    	distance=pi*dRadius;
    	//90° Distance
    	distance=distance/4;
    	
    	//Rotations per Motor 2.1 Wheel Diameter
    	rotations=distance/(pi*2.1);
    	degreescount=rotations*360;
    	
    	Motor.B.backward();
    	Motor.A.forward();
    	
    	//Rotate Angles
    	Motor.A.rotate((int)degreescount);
    }
    
    
    //Filtering Degree
	public static double getDegreeCalc(){
		
		//
		double degree;        
        double aux=0.0,e=0.0;		
		//Filter
    	aux=comp.getDegreesCartesian();
    	
    	if ((aux-last)<0)
    	{
    	last=0.0;
    	tau=0.4;
    	}
    	else
    	{
    	tau=(last-aux);
       	last=aux;
    	}
    	
    	e=(aux-last)*tau;
     	degree=aux+e;
    	
     	return degree;
    	
	}
	
	
	
	//Calculating 
	public static double CalculatePosition(Integration Iv,Integration Ip) {
	
	    
		double result;	

		//Obtaining Acceleration Value Axis X
		result=(double)accel.getXAccel();
		result=(result*9.81)/200;
		//Integration for obtaining speed
		result=Iv.addReading(result);
		//Integrating again for obtaining position
		result=Ip.addReading(result);
		
		if (result<0.0)
			result=0.01;
		return result;
		
	}
	
	

	
	
	
    public static void main(String[] args) throws Exception {
    	       

        double realmeasure;
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
        Motor.A.setSpeed(120);
        Motor.B.setSpeed(120);
             
        
        //while (!touch.isPressed())
        for(int i=0;i<12;i++)
        {
	
            
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
        	Motor.B.stop(true);
        	
        	//Wait to See
        	Thread.sleep(1000);
        	
        	//Rotating Robot
        	Motor.A.forward();
            Motor.B.forward();
        	
            
            //Going to front
            while (posChange<accumPos)
        	posChange=CalculatePosition(Ivel,Ipos);

            
            //Stop without Inertia
        	Motor.A.stop(true);
        	Motor.B.stop(true);
        	
        	
        	// Calculating Degrees
        	initialDegree=getDegreeCalc();        	
        	finalDegree=initialDegree+90.0;
        	
        	
        	//If InitialDegree is more than 270°
        	if (finalDegree>360)
        		finalDegree=finalDegree-360;     	
        	        	
         	System.out.print(initialDegree+" I.Deg. \n");
        	System.out.print(finalDegree+" F.Deg. \n");

        	Motor.A.stop(true);
            Motor.B.stop(true);

        	//Wait to See Screen
        	Thread.sleep(1000);
        	
        	
        	//Rotate
        	robotRotation();
     

        	
        }
 
    
    
    
    
    
    
    
    
    
    
    }
}