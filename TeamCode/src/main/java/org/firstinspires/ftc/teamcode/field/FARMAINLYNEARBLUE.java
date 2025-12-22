package org.firstinspires.ftc.teamcode.field;

import static org.firstinspires.ftc.teamcode.field.Field.Num_shots.THREE;
import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;
import static org.firstinspires.ftc.teamcode.field.Route.TeamElement.*;
import static org.firstinspires.ftc.teamcode.field.Route.numshot;

import com.acmerobotics.roadrunner.geometry.Pose2d;



public class FARMAINLYNEARBLUE {
    String TAG = "FARMAINLYNEARBLUE";
    Route route;
    private Field.Parks stackToBack;
    private Field.Parks pixelStack;

    private Route.TeamElement teamElement;
    private Field.Alliance alliance;

    public FARMAINLYNEARBLUE(Route constructorRoute) {
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

        route.addLocation(route.startSmallTriBlue, START, HEAD_LINEAR);
        route.addFunction(route::shootFar);
        route.addLocation(route.shootFarPosBLUE, SPLINE, HEAD_LINEAR, Math.toRadians(110));
        route.addEvent(Route.Action.WAIT,5);
        route.addFunction(route::shoot3Wiffles);
        route.addEvent(Route.Action.WAIT,1);



        route.addLocation(route.moveToLeverblue, LINE, HEAD_LINEAR, Math.toRadians(0));
        route.addFunction(route::intakeonandthreeTransitionsDown);
        route.addEvent(Route.Action.SLOW,25);
        route.addLocation(route.helpCollectLever5blue, LINE, HEAD_LINEAR, Math.toRadians(0));
        route.addFunction(route::shootWiffleClose);


        //Shoot 3 spot.
        route.addEvent(Route.Action.TANGENT, Math.toRadians(-50));
        route.addLocation(route.shootGateWhifflesBlue, SPLINE, HEAD_LINEAR, Math.toRadians(80));
        route.addFunction(route::intakeOff);
        route.shootMotif(1,route.shootGateWhifflesBlue );
        route.addFunction(route::intakeonandthreeTransitionsDown);
        route.addFunction(route::wheelOff);

        //intake and shoot 4
        route.addLocation(route.intakeGoalBlue, LINE, HEAD_LINEAR, Math.toRadians(0));
        route.addEvent(Route.Action.SLOW,0);
        route.addLocation(route.intakeGoalWhifflesBlue, LINE, HEAD_LINEAR, Math.toRadians(0));
        route.addFunction(route::shootWiffleClose);


        //SHOOT  WIFFLES
        route.addLocation(route.shootGoalWhifflesBlue, LINE, HEAD_LINEAR, Math.toRadians(0));
        route.addFunction(route::intakeOff);
        route.addFunction(route::shootWiffleClose);
        route.shootMotif(1,route.shootGoalWhifflesBlue );
        route.addFunction(route::intakeonandthreeTransitionsDown);
        route.addFunction(route::wheelOff);


        if (numshot==THREE) {
//            //potentially shoot slot 2
//            route.addLocation(route.moveToParkBlue, LINE, HEAD_LINEAR, Math.toRadians(0));
//            route.addEvent(Route.Action.SLOW,10);
//            route.addLocation(route.helpcollect5Blue, LINE, HEAD_LINEAR, Math.toRadians(0));
//            //shoot the park
//            route.addLocation(route.shootGateWhifflesBlue, LINE, HEAD_LINEAR, Math.toRadians(0));
//            route.addFunction(route::intakeOff);
//            route.addFunction(route::shootWiffleClose);
//            route.shootMotif(1,route.shootGateWhifflesBlue );
//            route.addFunction(route::intakeonandthreeTransitionsDown);
//            route.addFunction(route::wheelOff);
        }else {

            // route.shootMotif(2, );
        }

    }




    

//////////////////////////////////////////////////////////////////////////////////////////////////////



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


