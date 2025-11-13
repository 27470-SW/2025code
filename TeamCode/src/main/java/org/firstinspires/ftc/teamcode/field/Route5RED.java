package org.firstinspires.ftc.teamcode.field;

import static org.firstinspires.ftc.teamcode.field.Field.Num_shots.*;
import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;
import static org.firstinspires.ftc.teamcode.field.Route.TeamElement.*;
import static org.firstinspires.ftc.teamcode.field.Field.Parks.*;
import static org.firstinspires.ftc.teamcode.field.Route.numshot;

import com.acmerobotics.roadrunner.geometry.Pose2d;



public class Route5RED {
    String TAG = "ROUTE5RED";
    Route route;
    private Field.Parks stackToBack;
    private Field.Parks pixelStack;

    private Route.TeamElement teamElement;
    private Field.Alliance alliance;

    public Route5RED(Route constructorRoute) {
        route = constructorRoute;
    }

    public void makeTraj(PositionOption startPos, Field.Park_Pos parkPos, Field.Wiffle_Pos lastLocation) {
  /*
        this.stackToBack = stackToBack;
        if(firstLocation == PIXEL_CENTER){
            pixelStack = Field.Highways.CENTER;
        }else if(firstLocation == PIXEL_WALL){
            pixelStack = WALL;
        }else{
            pixelStack = DOOR;
        }
        this.teamElement = teamElement;
        this.alliance = alliance;
    */
       //  qualifierRoute(startPos,parkPos,firstLocation);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

        route.addLocation(route. redGoal, START, HEAD_LINEAR);
        route.addFunction(route::shootFar);
        route.addLocation(route. aroundPartner, LINE, HEAD_LINEAR, Math.toDegrees(0));
        route.addEvent(Route.Action.WAIT, 1.0);
        route.addFunction(route::shoot1);
        route.addEvent(Route.Action.WAIT, 1);
        route.addFunction(route::shoot2);
        route.addEvent(Route.Action.WAIT, 1);
        route.addFunction(route::shoot3);
        route.addEvent(Route.Action.WAIT, 0.35);
       // route.addFunction(route::transitonDown);
        route.addFunction(route::intakeOn);
        //Shoot preloaded whiffles.

        route.addLocation(route.intakeGate, LINE, HEAD_LINEAR, Math.toDegrees(0));
        route.addLocation(route.intakeGateWhiffles, LINE, HEAD_LINEAR, Math.toDegrees(0));
        route.addEvent(Route.Action.TANGENT, Math.toRadians(185));
        route.addLocation(route.shootGateWhiffles, SPLINE, HEAD_LINEAR);
        route.addFunction(route::intakeOff);
        route.addFunction(route::shoot1);
        route.addEvent(Route.Action.WAIT, 1);
        route.addFunction(route::shoot2);
        route.addEvent(Route.Action.WAIT, 1);
        route.addFunction(route::shoot3);
        route.addEvent(Route.Action.WAIT, 0.35);
        //route.addFunction(route::transitonDown);
        route.addFunction(route::intakeOn);
        //Shoot whiffles that are near the gate.
        if (numshot==THREE) {
            route.addLocation(route.intakeGoal, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addLocation(route.intakeGoalWhiffles, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addLocation(route.shootGoalWhiffles, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addFunction(route::intakeOff);
           // route.shootMotif(2, );
        }else {

            route.addLocation(route.intakeGoal, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addLocation(route.intakeGoalWhiffles, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addLocation(route.shootGoalWhiffles, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addFunction(route::intakeOff);
           // route.shootMotif(2, );
            route.addFunction(route::intakeOn);
            //Shoot whiffles that are near the goal.
            route.addLocation(route.intakePark, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addLocation(route.intakeParkWhiffles, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addLocation(route.shootParkWhiffles, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addFunction(route::intakeOff);
            //route.shootMotif(2, );
        }
        //Shoot whiffles near the parking spot.

    }


    private void deliverSample(){

        route.addFunction(route::deliverSample);
       route.addEvent(Route.Action.WAIT,0.95);
//        route.addFunction(route::moveArmToDrop );
//        route.addEvent(Route.Action.WAIT,0.2);
//        route.addFunction(route::openclaw);
//        route.addEvent(Route.Action.WAIT,0.3);
//        route.addFunction(route::moveArmTo90 );
//        route.addEvent(Route.Action.WAIT,0.2);
//        route.addFunction(route::minSlides);
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////
    private void goToBackdrop(Pose2d backdrop){
        if(stackToBack == WALL)
            viaWall(backdrop);
        else if(stackToBack == Field.Parks.Park2){
            viaCenter(backdrop);
        }
        else{//DOOR

            viaDoor(backdrop);
        }
    }


    private void viaWall(Pose2d backdrop){
        route.addLocation(route.byBlueLoadStation, SPLINE, HEAD_LINEAR, route.zero);
        route.makeNewTraj();
        route.addLocation(route.moveTowardsRedBackdropLft, LINE, HEAD_LINEAR,route.ninety);
        route.addFunction(route::armToDrop, 1.5);
        route.addLocation(backdrop, SPLINE, HEAD_LINEAR,route.ninety);
    }

    private void viaCenter(Pose2d backdrop){
        route.addEvent(Route.Action.TANGENT, route.twoseventy);
        route.addLocation(route.whatchamacallit, SPLINE, HEAD_LINEAR);
        route.addFunction(route::armToDrop, 1.5);
        route.addLocation(backdrop, LINE, HEAD_LINEAR);
    }

    private void viaDoor(Pose2d backdrop){
        route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR, route.twoseventy);
        route.addEvent(Route.Action.TANGENT, route.oneHundred);
        route.addFunction(route::armToDrop, 2);
        if(alliance == Field.Alliance.RED && teamElement == RIGHT){
            route.addLocation(backdrop, SPLINE, HEAD_LINEAR, route.ten);
        }else {
            route.addLocation(backdrop, SPLINE, HEAD_LINEAR, route.thirty);
        }
    }

    private void moveStartToLeft(){

    }






    protected void moveFromStart() {

        route.addLocation(route.start, LINE, HEAD_LINEAR);
        //addLocation(extra1, LINE, HEAD_LINEAR);
        //addLocation(Start, LINE, HEAD_LINEAR);
        //addLocation(awayFromStart, LINE, HEAD_LINEAR);;
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


