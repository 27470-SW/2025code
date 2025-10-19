package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.DECODE_Route.preLoadedCone.high;
import static com.example.meepmeeptesting.DECODE_Route.preLoadedCone.low;
import static com.example.meepmeeptesting.DECODE_Route.preLoadedCone.lowHigh;
import static com.example.meepmeeptesting.DECODE_Route.preLoadedCone.med;
import static com.example.meepmeeptesting.RobotConstants.MAX_LOOPS;
import static com.example.meepmeeptesting.Route.Heading.HEAD_CONSTANT;
import static com.example.meepmeeptesting.Route.Heading.HEAD_LINEAR;
import static com.example.meepmeeptesting.Route.Heading.HEAD_SPLINE;
import static com.example.meepmeeptesting.Route.Movement.LINE;
import static com.example.meepmeeptesting.Route.Movement.SPLINE;
import static com.example.meepmeeptesting.Route.Movement.START;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Locale;

public class TimeForPixelRun extends Route{
    DECODE_Route route;
    int count;

    public TimeForPixelRun(Route constructorRoute, Field.Highways pixelStack) {
        super(constructorRoute);


        finalizeTrajSeq();
    }

    public void makeTraj() {
       firstMovement();

    }
   public boolean isTime(double time){
        if (time + totalDur <= 30){
            count++;
            if(count < MAX_LOOPS)
                return true;
            else
                return false;
        }
        else{
            return false;
        }
   }

   /* public Boolean isTime(){
        if(count < 2)
        {
            count++;
            return true;
        }
        return false;
    }*/





    protected void startPos(){

    }

    protected void firstMovement()
    {
        route.addLocation(route.startCSRedHigh, START, HEAD_LINEAR);
        route.addLocation(route.purplePixelPlaceCenterTop,LINE, HEAD_SPLINE);
        route.addLocation(route.dropCenterPixel,LINE, HEAD_LINEAR);
        route.addEvent(Route.Action.TANGENT, Math.toRadians(-1500));
       route.addLocation(route.underRigging,SPLINE, HEAD_LINEAR);
     //  route.addEvent(Route.Action.TANGENT, Math.toRadians(-1500));
        route.addLocation(route.pickUpPixelStackRight,SPLINE, HEAD_LINEAR);
       // route.addEvent(Route.Action.WAIT, 0.3);
       route.addEvent(Route.Action.TANGENT, Math.toRadians(180));
        route.addLocation(route.underdoor,SPLINE, HEAD_CONSTANT);
        route.addLocation(route.dropCenterPixel,LINE, HEAD_CONSTANT);
        route.addLocation(route.underRigging,LINE, HEAD_SPLINE);
      //  route.addEvent(Route.Action.TANGENT, Math.toRadians(1000));
        route.addLocation(route.pickUpPixelStackCenter,SPLINE, HEAD_CONSTANT);
       // route.addLocation(route.plowSignalConeOnCenterPark,SPLINE, HEAD_CONSTANT);
    }

    protected void moveFromStart() {

        route.addLocation(route.start, LINE, HEAD_LINEAR);
        //addLocation(extra1, LINE, HEAD_LINEAR);
        //addLocation(Start, LINE, HEAD_LINEAR);
        //addLocation(awayFromStart, LINE, HEAD_LINEAR);
        //preLoadedConeJunc();
      /*addFunction( this::liftElvLowPoleJnct);
      addLocation(extra3, LINE, HEAD_LINEAR);
      addLocation(extra2, LINE, HEAD_SPLINE);
      addFunction( this::elvToConeStack);
      addEvent(Action.WAIT, 0.5);
      addFunction( this::openClaw);
      addLocation(extra3, LINE, HEAD_SPLINE);
      addLocation(preGrabCone[0], LINE, HEAD_SPLINE);
      addLocation(strafeGrabCone[0], LINE, HEAD_LINEAR);
      addFunction( this::closeClaw);
      addEvent(Action.WAIT, 0.3);*/
        //addFunction( this::liftElvMediumPoleJnct);
        //addLocation(toTurnBox, LINE, HEAD_LINEAR);
    }

    protected Pose2d awayFromStart;
    protected void liftElvGndJnct()
    {
        System.out.println("Lift Elevator to GND JNCT");
    }

    protected void liftElvLowPoleJnct()
    {
        System.out.println("Lift Elevator to LOW POLE JNCT");
    }

    protected void liftElvMediumPoleJnct()
    {
        System.out.println("Lift Elevator to MED POLE JNCT");
    }

    /* Lift Elevator to the Highest Level to prepare to drop Cone */
    protected void liftElvHighPoleJnct()
    {
        System.out.println("Lift Elevator to HIGH POLE JNCT");
    }

    /* Lift Elevator to the Highest Level to prepare to drop Cone */
    protected void liftElvConeStackLvlFive()
    {
        System.out.println("Lift Elevator to CONE STACK LVL 5");

    }

    protected void liftElvConeStackLvlFour()
    {
        System.out.println("Lift Elevator to CONE STACK LVL 4");

    }

    protected void liftElvConeStackLvlThree()
    {
        System.out.println("Lift Elevator to CONE STACK LVL 3");

    }

    protected void liftElvConeStackLvlTwo()
    {
        System.out.println("Lift Elevator to CONE STACK LVL 2");

    }

    protected void liftElvConeStackLvlOne()
    {
        System.out.println("Lift Elevator to CONE STACK LVL 1");

    }

    public enum preLoadedCone{low,med,high,lowHigh}
    public static DECODE_Route.preLoadedCone preLoadedConeJunc = low;
    public static DECODE_Route.preLoadedCone firstConeStackCone = med;
    public static DECODE_Route.preLoadedCone secondConeStackCone = lowHigh;
    public static DECODE_Route.preLoadedCone thirdConeStackCone = high;

    public void elvToConeStack()
    {
        switch (conestackNum)
        {
            case 1:
                liftElvConeStackLvlOne();
                break;
            case 2:
                liftElvConeStackLvlTwo();
                break;
            case 3:
                liftElvConeStackLvlThree();
                break;
            case 4:
                liftElvConeStackLvlFour();
                break;
            case 5:
                liftElvConeStackLvlFive();
                break;
            default:
                liftElvConeStackLvlOne();
                break;
        }
        conestackNum --;
    }

    public final static int INIT_CONE_STACK = 5;
    public static int conestackNum = INIT_CONE_STACK;

    protected void openClaw()
    {
        System.out.println("OPEN Claw");

    }

    /* Grab the cone in the stack or in the terminal */
    protected void closeClaw()
    {
        System.out.println("Close Claw");

    }

    protected void elvConeStackSecondOption() {
        setClawPos(.6);
        liftElvGndJnct();
        setClawPos(.9);
        liftElvMediumPoleJnct();
    }
    protected void setClawPos(double pos){
        System.out.printf(Locale.US,"set claw pos to %f",pos );
    }

   /*rotected final int sx;
    protected final int sy;
    protected int sh;
    protected int sf;
    protected int sr;
    protected final double flip;
    protected final double strtX;
    protected final double strtY;
    protected final double strtH;
    protected Field.Alliance alliance;
    protected PositionOption startPos;
    protected final double botBackToCtr;
    protected final double botSideToCtr;
    double rightSideLineUpToBorderAdjustment = 0.5;

    protected Pose2d moveFromStart;
    protected Pose2d start;*/

}


