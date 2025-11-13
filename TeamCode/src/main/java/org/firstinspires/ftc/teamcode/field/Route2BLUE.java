package org.firstinspires.ftc.teamcode.field;

import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;
import static org.firstinspires.ftc.teamcode.field.Route.TeamElement.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;



public class Route2BLUE {
    String TAG = "Route2Blue";
    Route route;
    private Field.Parks stackToBack;
    private Field.Parks pixelStack;

    private Route.TeamElement teamElement;
    private Field.Alliance alliance;

    public Route2BLUE(Route constructorRoute) {
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

        route.addLocation(route.startBlueFar, START, HEAD_LINEAR);
        route.addLocation(route.shootfaronred, LINE, HEAD_LINEAR, Math.toRadians(0));
        route.addEvent(Route.Action.WAIT, 2);
        route.addLocation(route.moveToPark, LINE, HEAD_LINEAR, Math.toRadians(0));
        route.addLocation(route.collect2, LINE, HEAD_LINEAR, Math.toRadians(0));
        route.addLocation(route.shootfaronred, LINE, HEAD_LINEAR, Math.toRadians(0));
        route.addEvent(Route.Action.WAIT, 0.1);
        route.addLocation(route.nearLeaver, LINE, HEAD_LINEAR, Math.toRadians(0));
        route.addLocation(route.collect3, LINE, HEAD_LINEAR, Math.toRadians(0));
        route.addEvent(Route.Action.TANGENT, Math.toRadians(190));
        route.addLocation(route.shootNear, SPLINE, HEAD_LINEAR, Math.toRadians(90));
        route.addEvent(Route.Action.WAIT, 0.1);
        route.addEvent(Route.Action.TANGENT, Math.toRadians(90));
        route.addLocation(route.moveToHumanPlayerZone, SPLINE, HEAD_LINEAR, Math.toRadians(0));
        route.addLocation(route.collect1, LINE, HEAD_LINEAR, Math.toRadians(0));
        route.addEvent(Route.Action.TANGENT, Math.toRadians(100));
        route.addLocation(route.shootNear2, SPLINE, HEAD_LINEAR, Math.toRadians(0));
        route.addEvent(Route.Action.WAIT, 0.1);


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


