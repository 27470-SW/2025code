package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Field.Highways.WALL;
import static com.example.meepmeeptesting.Field.Num_shots.THREE;
import static com.example.meepmeeptesting.Route.Heading.HEAD_LINEAR;
import static com.example.meepmeeptesting.Route.Movement.LINE;
import static com.example.meepmeeptesting.Route.Movement.SPLINE;
import static com.example.meepmeeptesting.Route.Movement.START;
import static com.example.meepmeeptesting.Route.TeamElement.RIGHT;
import static com.example.meepmeeptesting.Route.numshot;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Route5RED {
    Route route;
    private Field.Highways stackToBack;
    private Field.Highways pixelStack;

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
        route.addFunction(route::wheelOn);
        route.addLocation(route. aroundPartner, LINE, HEAD_LINEAR, Math.toDegrees(0));
        route.addEvent(Route.Action.WAIT, 1.0);
        route.addFunction(route::shoot1);
        route.addEvent(Route.Action.WAIT, 1);
        route.addFunction(route::shoot2);
        route.addEvent(Route.Action.WAIT, 1);
        route.addFunction(route::shoot3);
        route.addEvent(Route.Action.WAIT, 0.35);
        route.addFunction(route::transitonDown);
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
        route.addFunction(route::transitonDown);
        route.addFunction(route::intakeOn);
        //Shoot whiffles that are near the gate.
        if (numshot==THREE) {
            route.addLocation(route.intakeGoal, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addLocation(route.intakeGoalWhiffles, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addLocation(route.shootGoalWhiffles, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addFunction(route::intakeOff);
            route.shootMotif();
        }else {

            route.addLocation(route.intakeGoal, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addLocation(route.intakeGoalWhiffles, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addLocation(route.shootGoalWhiffles, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addFunction(route::intakeOff);
            route.shootMotif();
            route.addFunction(route::intakeOn);
            //Shoot whiffles that are near the goal.
            route.addLocation(route.intakePark, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addLocation(route.intakeParkWhiffles, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addLocation(route.shootParkWhiffles, LINE, HEAD_LINEAR, Math.toDegrees(0));
            route.addFunction(route::intakeOff);
            route.shootMotif();
        }
        //Shoot whiffles near the parking spot.

    }

    private void pickupSampleFromTape() {
        route.addFunction(route::moveArmToPickup);
        route.addEvent(Route.Action.WAIT, 0.2);
        route.addFunction(route::closeClaw);
        route.addEvent(Route.Action.WAIT, 0.2);
        route.addFunction(route::moveArmTo90);
        route.addFunction(route::maxSlides);
    }

    private void deliverSample() {
        route.addFunction(route::moveArmToDrop);
        route.addEvent(Route.Action.WAIT, 0.2);
        route.addFunction(route::openclaw);
        route.addEvent(Route.Action.WAIT, 0.2);
        route.addFunction(route::moveArmTo90);
        route.addEvent(Route.Action.WAIT, 0.2);
        route.addFunction(route::minSlides);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    private void goToBackdrop(Pose2d backdrop) {
        if (stackToBack == WALL)
            viaWall(backdrop);
        else if (stackToBack == Field.Highways.Park2) {
            viaCenter(backdrop);
        } else {//DOOR

            viaDoor(backdrop);
        }
    }


    private void viaWall(Pose2d backdrop) {
        route.addLocation(route.byBlueLoadStation, SPLINE, HEAD_LINEAR, route.zero);
        route.makeNewTraj();
        route.addLocation(route.moveTowardsRedBackdropLft, LINE, HEAD_LINEAR, route.ninety);
        route.addFunction(route::armToDrop, 1.5);
        route.addLocation(backdrop, SPLINE, HEAD_LINEAR, route.ninety);
    }

    private void viaCenter(Pose2d backdrop) {
        route.addEvent(Route.Action.TANGENT, route.twoseventy);
        route.addLocation(route.whatchamacallit, SPLINE, HEAD_LINEAR);
        route.addFunction(route::armToDrop, 1.5);
        route.addLocation(backdrop, LINE, HEAD_LINEAR);
    }

    private void viaDoor(Pose2d backdrop) {
        route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR, route.twoseventy);
        route.addEvent(Route.Action.TANGENT, route.oneHundred);
        route.addFunction(route::armToDrop, 2);
        if (alliance == Field.Alliance.RED && teamElement == RIGHT) {
            route.addLocation(backdrop, SPLINE, HEAD_LINEAR, route.ten);
        } else {
            route.addLocation(backdrop, SPLINE, HEAD_LINEAR, route.thirty);
        }
    }


    private void qualifierRoute(PositionOption startPos, Field.Highways parkPos, Field.Wiffle_Pos firstLocation) {
        if (alliance == Field.Alliance.RED) {
            switch ((Field.StartPos) startPos) {
                case START_SAMPLES:
                    route.addLocation(route.start, SPLINE, HEAD_LINEAR);
                    switch (teamElement) {
                        case LEFT:
                            // Red Left Backdrop (7252)
//                            route.addLocation(route.moveAwayFromWallRedBackdrop, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelRedLeftTapeBackdropAdj, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelRedLeftTapeBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addFunction(route::outPurplePixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::armToDrop);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.moveAwayFromLRedBackdropTape, LINE, HEAD_LINEAR);
////                            route.addMovement(TURN, -0.9);
//                            route.addEvent(Route.Action.WAIT, 0.3);
//                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.1);
////                            route.addMovement(TURN, 0.7);
//                            route.addEvent(Route.Action.WAIT, 0.3);
//                            route.addLocation(route.dropOnBackdropRedLeftBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.1);
////                            route.addMovement(TURN, -0.5);
//                            route.addEvent(Route.Action.WAIT, 0.1);
                            // my old route
                            route.addEvent(Route.Action.TANGENT, route.oneFifteen);
//                            route.addLocation(route.dropPixelRedLeftTapeBackdropAdj,SPLINE,HEAD_LINEAR, 180);
                            route.addLocation(route.dropPixelRedLeftTapeBackdrop, LINE, HEAD_LINEAR, route.twofifity);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.15);
                            route.addFunction(route::outPurplePixel);
                            route.addEvent(Route.Action.WAIT, .55);
//                            route.addEvent(Route.Action.TANGENT, route.ninety);
                            route.addFunction(route::armToDrop);
                            route.addEvent(Route.Action.WAIT, .2);
                            route.addLocation(route.dropOnBackdropRedLeftBackdrop, LINE, HEAD_LINEAR, Math.toRadians(100));
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .75);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, .2);
                            break;
                        case CENTER:
                            // Red Center Backdrop (7252)
//                            route.addLocation(route.moveAwayFromWallRedBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addLocation(route.dropPixelRedCenterBackTapeAdj, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelRedCenterTapeBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPurplePixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::armToDrop);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.moveAwayFromCRedBackdropTape, LINE, HEAD_LINEAR);
////                            route.addMovement(TURN, -0.9);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropOnBackdropRedCenterBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
////                            route.addMovement(TURN, -0.5);
//                            route.addEvent(Route.Action.WAIT, 0.2);
                            // my old route
                            route.addLocation(route.dropPixelRedCenterTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos, .3);
                            route.addEvent(Route.Action.WAIT, .3);
                            route.addFunction(route::outPurplePixel);
                            route.addEvent(Route.Action.WAIT, .55);
//                            route.addEvent(Route.Action.TANGENT, route.thirty);
                            route.addFunction(route::armToDrop);
                            route.addLocation(route.dropOnBackdropRedCenterBackdrop, SPLINE, HEAD_LINEAR, route.oneFifteen);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .5);
                            break;
                        case RIGHT:
                            // Red Right Backdrop (7252)
//                            route.addLocation(route.moveAwayFromWallRedBackdrop, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelRedRightTapeBackdropAdj, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelRedRightTapeBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::outPurplePixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::armToDrop);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.moveAwayFromRRedBackdropTape, LINE, HEAD_LINEAR);
////                            route.addMovement(TURN, -0.9);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropOnBackdropRedRightBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
////                            route.addMovement(TURN, -0.5);
//                            route.addEvent(Route.Action.WAIT, 0.2);
                            //my old route
                            route.addLocation(route.dropPixelRedRightTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos, .3);
                            route.addEvent(Route.Action.WAIT, .3);
                            route.addFunction(route::outPurplePixel);
                            route.addEvent(Route.Action.WAIT, .55);
//                        route.addEvent(Route.Action.TANGENT, route.thirty);
                            route.addFunction(route::armToDrop);
                            route.addLocation(route.dropOnBackdropRedRightBackdrop, SPLINE, HEAD_LINEAR, route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .35);

//                        route.addFunction(route::outPixel);
//                        route.addEvent(Route.Action.WAIT, .5);
                            break;
                    }
                    break;
                case START_SPECIMENS:
                    route.addLocation(route.start, START, HEAD_LINEAR);
                    switch (teamElement) {
                        case LEFT:
                            // Red Left Stacks (7252)
//                            route.addLocation(route.moveAwayFromWallRedStacks, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelRedLeftTapeStacks, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addFunction(route::outPurplePixel);
//                            route.addEvent(Route.Action.WAIT, 0.3);
//                            if (firstLocation == BACKDROP) {
//                                route.addLocation(route.moveFromRedLeftTapeStacks, LINE, HEAD_LINEAR);
//                            } else if (firstLocation == PIXEL_WALL) {
//                                route.addLocation(route.pickUpPixelStackLeft, LINE, HEAD_LINEAR);
//                                route.addFunction(route::armToIntake);
//                                route.addEvent(Route.Action.WAIT, 1);
//                                route.addFunction(route::intakes);
//                                route.addEvent(Route.Action.WAIT, .35);
//                                route.addFunction(route::armDropSpikePos);
//                                route.addFunction(route::outFrontPixel);
//                                route.addEvent(Route.Action.WAIT, .2);
//                                route.addLocation(route.moveFromRedLeftTapeStacks, LINE, HEAD_LINEAR);
//                                route.addLocation(route.dropPixelRedLeftTapeStacks, LINE, HEAD_LINEAR);
//                                route.addMovement(TURN, .7);
//                            }
//                            route.addLocation(route.byBlueLoadStation, LINE, HEAD_LINEAR);
//                            route.addLocation(route.moveTowardsRedBackdropLft, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armToDrop);
//                            route.addLocation(route.moveTowardsRedBackdropHdAdjLft, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropOnBackdropRedLeftStacks, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::armToDropHigher);
//                            route.addLocation(route.dropOnBackdropRedLeftStacksHi, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.75);
//                            route.addLocation(route.moveTowardsRedBackdropHdAdj, LINE, HEAD_LINEAR);
                            // my old route

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
        }
    }
}



