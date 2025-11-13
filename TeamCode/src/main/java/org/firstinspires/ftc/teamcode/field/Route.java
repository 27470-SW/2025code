package org.firstinspires.ftc.teamcode.field;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.image.ITD_Detector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.MecanumBot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import static org.firstinspires.ftc.teamcode.field.Route.Movement.LINE;
import static org.firstinspires.ftc.teamcode.opModes.Decode_Auton.autonDebug;

import static org.firstinspires.ftc.teamcode.robot.RobotConstants.CLOSE_SHOOTER_DIST;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.MAX_SHOOTER_DIST;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.MIN_TRAJ_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.TrajEnum.*;
import static org.firstinspires.ftc.teamcode.robot.Shooter.BALL_CHOICE.*;


@SuppressWarnings("unused")
public class Route
{
     private static final String TAG = "SJH_RTE";
    protected static final boolean VERBOSE = RobotConstants.logVerbose;

     protected static TrajectoryVelocityConstraint defVelLim =
       RobotConstants.defVelConstraint;
     protected static TrajectoryAccelerationConstraint defAccelLim =
       RobotConstants.defAccelConstraint;

     protected static TrajectoryVelocityConstraint wobVelLim =
       RobotConstants.slwVelConstraint;
     protected static TrajectoryAccelerationConstraint wobAccelLim =
       RobotConstants.slwAccelConstraint;

    protected static final boolean strafeDropX = false;

//    protected enum routeNames{
//        P1,
//        P2,
//        P3,
//        P4
//    }
//
//    private Map<routeNames, >

    //default; do not use
    public Route(Route copyRoute){
        this(copyRoute.robot, copyRoute.startPos, copyRoute.parkPos, copyRoute.lastLocation, copyRoute.motif, copyRoute.numshot);
    }
    public Route(MecanumBot robot){
       this(robot, Field.Start_Pos.values()[0], Field.Park_Pos.values()[0], Field.Wiffle_Pos.values()[0], Field.Motif.values()[0], Field.Num_shots.values()[0]);

    }

     public Route(MecanumBot robot,
             PositionOption startPos,
             Field.Park_Pos parkPos,
             Field.Wiffle_Pos firstLocation,
             Field.Motif  motif,
             Field.Num_shots num_shot)
    {

           this.robot = robot;
           this.startPos = startPos;
           this.parkPos  = parkPos;
           this.lastLocation = firstLocation;
           this.numshot= num_shot;
           this.motif = motif;

           botLen = RobotConstants.BOT_LEN;
           botWid = RobotConstants.BOT_WID;

           double rightSideLineUpToBorderAdjustment = 0.5;


         botBackToCtr = botLen / 2.0;
         botSideToCtr = botWid / 2.0;

         hubRad = 9.0;
         double backOffset = 0.0;

         totalDur = 0;

         /* All points of interests will be based off of quadrant I - positive x, positive y */
         /* Reflection over the x-axis will be done for the alliance side */
         /* Reflection over the y-axis will be done on which side you line up */
         /* Blue Left = Quad I, Blue Right = Quad II, Red Right =  */
         /* For headings: Counter clockwise is positive, Clockwise is negative */
         if (alliance == Field.Alliance.BLUE)
		 {
             /* y scalar value  */
             /* Blue side is in quadrant I & II of the 2D coordinate system */
             sy = 1;
             /* Bot should point south or towards the negative Y direction */
             /* point the bot in the clockwise direction */
             sh = -1;
             sf = 0;
             sx = -1;
             flip = Math.toRadians(180);

                 sr = 0;

         }
		 else
		 {
             /* Red side is in quadrant III & IV of the 2D coordinate system */
             sy = 1;
             /* Bot should point north or in the positive Y direction on red side */
             /* point the bot in the counter clockwise direction */
             sh = 1;
             sf = -1;
             sx =1;
             flip = Math.toRadians(0);


                 /* Red Left quadrant III */
                 sr = 0;


         }



             strtY = -1.5 * Decode_Field.tileWidth;


         strtX =  3.0f * Decode_Field.tileWidth - botBackToCtr;

         strtH = Math.toRadians(180.0);

         start = new Pose2d(sx * strtX, sy * strtY, flip + sh* strtH);
         ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //Put new poses here
        startSample = new Pose2d(sx * 5, 63, flip + sh*Math.toRadians(270));
        //specimen points
        blueFar  = new Pose2d(sx * -11, -60, flip + sh*Math.toRadians(270));
        shootPreWhiffles  = new Pose2d(sx * -14, -57, flip + sh*Math.toRadians(115));
        redWall  = new Pose2d(sx * 36, 62, flip + sh*Math.toRadians(270));
        aroundPartner = new Pose2d(sx * 13, 43.1, flip + sh*Math.toRadians(30));
        shootZonefornear = new Pose2d(sx * 30, 29, flip + sh*Math.toRadians(53));
        shootZonefornearblue = new Pose2d(sx * -30, 20, flip + sh*Math.toRadians(-53));

        intakeGoal = new Pose2d(sx * 31.3, 8, flip + sh*Math.toRadians(90));
        intakeGoalWhiffles = new Pose2d(sx * 55 , 8, flip + sh*Math.toRadians(90));
        shootGoalWhiffles = new Pose2d(sx * 13, 43.1, flip + sh*Math.toRadians(30));
        intakeGate = new Pose2d(sx * 31.3, -14, flip + sh*Math.toRadians(90));
        intakeGateWhiffles = new Pose2d(sx * 52, -14, flip + sh*Math.toRadians(90));
        shootGateWhiffles = new Pose2d(sx * 24, 43.1, flip + sh*Math.toRadians(30));
        intakePark = new Pose2d(sx * 31.3, -37, flip + sh*Math.toRadians(90));
        intakeParkWhiffles = new Pose2d(sx * 52, -37, flip + sh*Math.toRadians(90));
        shootParkWhiffles = new Pose2d(sx * 13, 43.1, flip + sh*Math.toRadians(30));
        redGoal = new Pose2d(sx * 43.8, 57.5, flip + sh*Math.toRadians(217));
        blueGoal = new Pose2d(sx * -43.8, 57.5, flip + sh*Math.toRadians(143));
        blueWall  = new Pose2d(sx * -36, 62, flip + sh*Math.toRadians(270));
        aroundPartnerBlue = new Pose2d(sx * -13, 43.1, flip + sh*Math.toRadians(150));
        intakeGateBlue = new Pose2d(sx * -13, -5, flip + sh*Math.toRadians(270));
        intakeGateWhifflesBlue = new Pose2d(sx * -53, -5, flip + sh*Math.toRadians(270));
        shootGateWhifflesBlue = new Pose2d(sx * -13, 43.1, flip + sh*Math.toRadians(150));
        aroundWhiffles = new Pose2d(sx * -13, -5, flip + sh*Math.toRadians(150));
        intakeGoalBlue = new Pose2d(sx * -13, 17.3, flip + sh*Math.toRadians(270));
        shootGoalWhifflesBlue = new Pose2d(sx * -13, 43.1, flip + sh*Math.toRadians(150));
        intakeGoalWhifflesBlue = new Pose2d(sx * -54 , 17.3, flip + sh*Math.toRadians(270));
        intakeParkBlue = new Pose2d(sx * -13, -29.4, flip + sh*Math.toRadians(270));
        intakeParkWhifflesBlue = new Pose2d(sx * -51, -29.4, flip + sh*Math.toRadians(270));
        shootParkWhifflesBlue = new Pose2d(sx * -13, 43.1, flip + sh*Math.toRadians(150));
        //ITD poses
        startSmallTri = new Pose2d(sx * 15, -61, flip + sh*Math.toRadians(70));
        simpleShootFar = new Pose2d(sx * 17, -56, flip + sh*Math.toRadians(70));

        startSmallTriBlue = new Pose2d(sx * -15, -61, flip + sh*Math.toRadians(90));
        shootFarPosBLUE = new Pose2d(sx * -15, -56, flip + sh*Math.toRadians(110));

        shootFarPos = new Pose2d (sx * 15, -58, flip + sh*Math.toRadians(70));
        shootFarPosB = new Pose2d (sx * -16, -58, flip + sh*Math.toRadians(290));
        shootNearPos = new Pose2d (sx * 10, 1, flip + sh*Math.toRadians(70));
        shootNearPosB = new Pose2d (sx * 10, 1, flip + sh*Math.toRadians(290));
        PrepPosIntake1 = new Pose2d(sx * 58, -50 , flip + sh*Math.toRadians(45));
        PrepPosIntake1BLUE = new Pose2d(sx * -58, -50 , flip + sh*Math.toRadians(-75));

        intookpos1 = new Pose2d(sx * 60.3, -59 , flip + sh*Math.toRadians(0));
        intookpos1BLUE = new Pose2d(sx * -59, -60 , flip + sh*Math.toRadians(-45));
        intookpos1BLUEDONE = new Pose2d(sx * -59, -60 , flip + sh*Math.toRadians(-40));

        intakepos2 = new Pose2d(sx * 25, -40 , flip + sh*Math.toRadians(90));
        intakepos2B = new Pose2d(sx * -29, -28 , flip + sh*Math.toRadians(270));
        intakeloading2 = new Pose2d(sx * 50, -40 , flip + sh*Math.toRadians(90));
        intakeloading2B = new Pose2d(sx * -49, -28 , flip + sh*Math.toRadians(270));
        intakepos3 = new Pose2d(sx * 25, -17 , flip + sh*Math.toRadians(90));
        intakepos3B = new Pose2d(sx * -29, -5 , flip + sh*Math.toRadians(270));
        intakeloading3 = new Pose2d(sx * 50, -17 , flip + sh*Math.toRadians(90));
        intakeloading3B = new Pose2d(sx * -49, -5 , flip + sh*Math.toRadians(270));
        shootPos3 = new Pose2d(sx * -5, 18 , flip + sh*Math.toRadians(90));
        intakepos4 = new Pose2d(sx * 25, 7 , flip + sh*Math.toRadians(90));
        intakeloading4 = new Pose2d(sx * 50, 7 , flip + sh*Math.toRadians(90));
        preleverwiffle= new Pose2d(sx * 15, -17 , flip + sh*Math.toRadians(90));
        intakewifflelever = new Pose2d(sx * 52, -16 , flip + sh*Math.toRadians(90));
        hitlever = new Pose2d(sx * 54, -11 , flip + sh*Math.toRadians(90));
        prenearpos = new Pose2d (sx * 17,-10,flip +sh*Math.toRadians(40));
        nearpos = new Pose2d (sx * 15,37,flip +sh*Math.toRadians(35));
        pregotoparkwiffle = new Pose2d (sx * 15,-40,flip +sh*Math.toRadians(90));
        gotoparkwiffle = new Pose2d (sx * 50,-40,flip +sh*Math.toRadians(90));
        pregotogoalwiffles= new Pose2d (sx * 17,6,flip +sh*Math.toRadians(90));
        gotogoalwiffles= new Pose2d (sx * 50,6,flip +sh*Math.toRadians(90));

        parkInside1Red = new Pose2d(sx * 17,37,flip +sh*Math.toRadians(90));
        parkInside1Blue = new Pose2d(sx * -17,37,flip +sh*Math.toRadians(90));

        parkInside2Red = new Pose2d(sx * 15,-18,flip +sh*Math.toRadians(90));
        parkInside2Blue = new Pose2d(sx * -15,-18,flip +sh*Math.toRadians(90));

        parkOutside1Red =new Pose2d(sx * 30,58,flip +sh*Math.toRadians(90));
        parkOutside1Blue =new Pose2d(sx * -30,58,flip +sh*Math.toRadians(90));

        parkOutside2Red =new Pose2d(sx * 52,30,flip +sh*Math.toRadians(90));
        parkOutside2BLUE =new Pose2d(sx * -52,30,flip +sh*Math.toRadians(90));
        tempParkRed = new Pose2d(sx * 57,-50,flip +sh*Math.toRadians(90));
        tempParkBlue = new Pose2d(sx * -30,-40,flip +sh*Math.toRadians(50));

        startRedFar = new Pose2d(sx * 20, -60, flip + sh*Math.toRadians(90));
        moveToPark = new Pose2d (sx * 25, -37, flip + sh*Math.toRadians(90));
        collect2 = new Pose2d (sx * 60, -37, flip + sh*Math.toRadians(90));
        shootfaronred = new Pose2d (sx * 20, -52, flip + sh*Math.toRadians(60));
        nearLeaver = new Pose2d (sx * 17, -15, flip + sh*Math.toRadians(90));
        collect3 = new Pose2d (sx * 54, -15, flip + sh*Math.toRadians(90));
        shootNear = new Pose2d (sx * 15, 37, flip + sh*Math.toRadians(25));
        shootNearForSpline = new Pose2d (sx * 30, 37, flip + sh*Math.toRadians(30));
        moveToHumanPlayerZone = new Pose2d (sx * 61, -40, flip + sh*Math.toRadians(360));
        collect1 = new Pose2d (sx * 61, -60, flip + sh*Math.toRadians(360));
        shootNear2 = new Pose2d (sx * 30, 37, flip + sh*Math.toRadians(30));
        startSpecimenSide = new Pose2d(sx * -5, 63.25, flip + sh*Math.toRadians(270));
		startSpecimen = new Pose2d(sx * -5, 63, flip + sh*Math.toRadians(270));
        specimenPark1 = new Pose2d(sx * -61, 62, flip + sh*Math.toRadians(270));
        specimen3 = new Pose2d(sx * -59.01, 55, flip + sh*Math.toRadians(245));
        specimen2 = new Pose2d(sx * -59, 55, flip + sh*Math.toRadians(270));
        dropPreSpecimen = new Pose2d(sx * -5, 35, flip + sh*Math.toRadians(270));
        forward = new Pose2d(sx * -5, 35, flip + sh*Math.toRadians(270));
        startCSRedHigh = new Pose2d(sx * 63.25, 16, flip + sh*Math.toRadians(180));
        specimen1PosPosToPush = new Pose2d(sx * -27.7, 35, flip + sh*Math.toRadians(270));
        specimen1PosToPush = new Pose2d(sx * -48, 10, flip + sh*Math.toRadians(270));
       specimen1 = new Pose2d(sx * -48,55, flip + sh*Math.toRadians(270));
       corner = new Pose2d(sx * -55,55, flip + sh*Math.toRadians(310));
        corner2 = new Pose2d(sx * -55,55, flip + sh*Math.toRadians(310));
        corner3 = new Pose2d(sx * -55,55, flip + sh*Math.toRadians(310));
        startCSRedLow = new Pose2d(sx*63.25, -32, flip + sh*Math.toRadians(180));
        startCBlueHigh = new Pose2d(sx * -63.25, 16, flip + sh*Math.toRadians(0));
        startCSBlueLow = new Pose2d(sx * -63.25, -15.5, flip + sh*Math.toRadians(0));
         purplePixelPlaceCenterTop = new Pose2d(sx * 28, 23.5, flip + sh*Math.toRadians(230));
         dropCenterPixel = new Pose2d(sx * 6, 65, flip + sh*Math.toRadians(270));
        hangSpecimen = new Pose2d(sx * 5, 44.9, flip + sh*Math.toRadians(270));
        moveBackFromSpecimen = new Pose2d(sx * 9, 55, flip + sh*Math.toRadians(270));
        sample1 = new Pose2d(sx * 47, 53.5, flip + sh*Math.toRadians(270));
        deliverSampleToBasket = new Pose2d(sx * 55, 61.25 , sh*Math.toRadians(50));
        deliverSample2ToBasket = new Pose2d(sx * 57.5, 64.2, sh*Math.toRadians(45));
        deliverSample3ToBasket = new Pose2d(sx * 56.5, 66.5, sh*Math.toRadians(53));
        sample2 = new Pose2d(sx * 54, 54.5, flip + sh*Math.toRadians(-100));// x is 58 y is 53.5 heading is -119.9
        rotateToSample2 = new Pose2d(sx * 55, 56, flip + sh*Math.toRadians(-90));
        turnStraight = new Pose2d (sx * 56, 54, flip + sh*Math.toRadians(-90));
        sample3 = new Pose2d(sx * 41.6, 27, flip + sh*Math.toRadians(-4));
        park = new Pose2d(sx * 28, 10, flip + sh*Math.toRadians(-180));
        positionToPark = new Pose2d(sx * 40,10 , flip + sh*Math.toRadians(-90));
        waitForHumanPlayer = new Pose2d(sx * -48, 48, flip + sh*Math.toRadians(89.9));
		waitForHumanPlayer2 = new Pose2d(sx * -46, 45.75, flip + sh*Math.toRadians(89.9));
        waitForHumanPlayer3 = new Pose2d(sx * -47.25, 43.5, flip + sh*Math.toRadians(89.9));
        park2 = new Pose2d(sx * -70, 63, flip + sh*Math.toRadians(-90));
        subPos = new Pose2d(sx * -5, 35, flip + sh*Math.toRadians(270));
        startRedWall = new Pose2d(sx * 30, 62, flip + sh*Math.toRadians(270));
        startBlueWall = new Pose2d(sx * -30, 62, flip + sh*Math.toRadians(270));
        shootNearBlue = new Pose2d(sx * -25, 50, flip + sh*Math.toRadians(150));
        goNear2Blue = new Pose2d(sx * -25, -30, flip + sh*Math.toRadians(270));
        collect2Blue = new Pose2d(sx * -55, -30, flip + sh*Math.toRadians(270));
        goNear3Blue = new Pose2d(sx * -25, -5, flip + sh*Math.toRadians(270));
        collect3Blue = new Pose2d(sx * -55, -5, flip + sh*Math.toRadians(270));
        goNear1Blue = new Pose2d(sx * -61, -43, flip + sh*Math.toRadians(180));
        collect1Blue = new Pose2d(sx * -61, -60, flip + sh*Math.toRadians(180));
        shootNearBlueSpline = new Pose2d(sx * -30, 40, flip + sh*Math.toRadians(150));
        startBlueFar= new Pose2d(sx * -20, -55, flip + sh*Math.toRadians(90));
        dropCenterPixelAdj = new Pose2d(sx * 27, 57.75, flip + sh*Math.toRadians(270));
        dropCenterPixelStacksAdj = new Pose2d(sx * 20, 50, flip + sh*Math.toRadians(270));
        dropCenterPixelLeftAdj = new Pose2d(sx *31.5, 50.35, flip + sh*Math.toRadians(270));
        dropCenterPixelCenterAdjBlue = new Pose2d(sx *35, 46, flip + sh*Math.toRadians(270));
        dropCenterPixelLeftAdjBlue = new Pose2d(sx *35.5, 43, flip + sh*Math.toRadians(270));
        dropCenterPixelLeftAdjRed= new Pose2d(sx *30, 51.35, flip + sh*Math.toRadians(270));
        dropCenterPixelRightAdjBlue = new Pose2d(sx *30, 54, flip + sh*Math.toRadians(270));
         dropCenterPixelBackwards = new Pose2d(sx * 32.5, 48, flip + sh*Math.toRadians(270));
        dropLeftPixelBackwardsBlue = new Pose2d(sx * 34.25, 63.5, flip + sh*Math.toRadians(270));
        dropRightPixelBackwardsRed = new Pose2d(sx * 40.5, 62.35, flip + sh*Math.toRadians(270));
        dropRightPixelBackwardsBlue = new Pose2d(sx * 25, 56.5, flip + sh*Math.toRadians(270));
        dropLeftPixelBackwardsRed = new Pose2d(sx * 27, 61, flip + sh*Math.toRadians(270));
        dropCenterPixelBackwardsBlue = new Pose2d(sx * 31.75, 56, flip + sh*Math.toRadians(270));
        dropCenterPixelBackwardsRed = new Pose2d(sx * 33.5, 61.5, flip + sh*Math.toRadians(270));
        dropCenterPixelBackwardsRedBd = new Pose2d(sx * 36, 59.5, flip + sh*Math.toRadians(270));
        dropRightPixelBackwardsRedBd = new Pose2d(sx * 42.5, 57.5, flip + sh*Math.toRadians(270));
        dropLeftPixelBackwardsRedBd = new Pose2d(sx * 17.5, 56.5, flip + sh*Math.toRadians(270));

        dropCenterPixelBackwardsLeftBackdrop = new Pose2d(sx * 34.5, 40, flip + sh*Math.toRadians(270));
        whatchamacallit = new Pose2d(sx * 36, -36, flip + sh*Math.toRadians(270));
        pickUpPixelStackLeft = new Pose2d(sx * 35, -62, flip + sh*Math.toRadians(-90));
        pickUpPixelStackLeftBlue = new Pose2d(sx * 35, -62, flip + sh*Math.toRadians(-90));
        pickUpPixelStackLeftLeft = new Pose2d(sx * 36.5, -63.5, flip + sh*Math.toRadians(-90));
        pickUpPixelStackLeftCenter = new Pose2d(sx * 40, -58, flip + sh*Math.toRadians(-90));
        dropLeftPixel = new Pose2d(sx * 33, 53.8, flip + sh* Math.toRadians(270));
        dropRightPixelBlue = new Pose2d(sx * 27, 52.5, flip + sh* Math.toRadians(270));
        dropRightPixel = new Pose2d(sx * 47, 54, flip + sh*Math.toRadians(270));
        dropRightPixelCircuit = new Pose2d(sx * 45, 54.5, flip + sh*Math.toRadians(270));
        dropLeftPixelBlue = new Pose2d(sx * 45, 53, flip + sh*Math.toRadians(270));
        pickUpPixelStackRight = new Pose2d(sx * 34.5, -59, flip + sh*Math.toRadians(-90));
        pickUpPixelStackRightCircuit = new Pose2d(sx * 36, -58.5, flip + sh*Math.toRadians(-90));
        pickUpPixelStackRightRedLeft = new Pose2d(sx * 31, -57, flip + sh*Math.toRadians(-90));
        pickUpPixelStackRightCircuitBlue = new Pose2d(sx * 30, -59, flip + sh*Math.toRadians(-90));
        pickUpPixelStackRightCircuitBlueCenter = new Pose2d(sx * 30.5, -59, flip + sh*Math.toRadians(-90));
        pickUpPixelStackRightCircuitBlueRight = new Pose2d(sx * 33.5, -55, flip + sh*Math.toRadians(-90));
        pickUpPixelStackRightLeft = new Pose2d(sx * 35, -61, flip + sh*Math.toRadians(-90));
        pickUpPixelStackCenter = new Pose2d(sx * 12, -59, flip + sh*Math.toRadians(-90));
        pickUpPixelStackCenterBlue = new Pose2d(sx * 17, -58, flip + sh*Math.toRadians(-90));
        backAwayBackDrop = new Pose2d(sx * 35, 43, flip + sh*Math.toRadians(180));
        underRigging = new Pose2d(sx * 66, -43, flip + sh*Math.toRadians(270));
        pointForRobotToPixelStack = new Pose2d(sx * 59, -42, flip + sh*Math.toRadians(270));
        pointForRobotToPixelStackBlue = new Pose2d(sx * 62, -24, flip + sh*Math.toRadians(270));
        aboveRigging = new Pose2d(sx * 61, 3, flip + sh*Math.toRadians(270));
        aboveRiggingBackwards = new Pose2d(sx * 61, 8, flip + sh*Math.toRadians(90));
        underCenter = new Pose2d(sx * 36, -9, flip + sh*Math.toRadians(270));
        pointcuzwhynot = new Pose2d(sx * 36, -37, flip + sh*Math.toRadians(270));
        underdoor = new Pose2d(sx*11, -63, flip + sh*Math.toRadians(270));
        underMiddlePoint = new Pose2d(sx*35, -22, flip + sh*Math.toRadians(270));
        bottom =new Pose2d( sx*30,24, flip + sh*Math.toRadians(0));
        middle=new Pose2d( sx* 25,18, flip + sh*Math.toRadians(180));
        top=new Pose2d( sx*35,21, flip + sh*Math.toRadians(90));
        parkDoor = new Pose2d(sx*10, 44.5, flip + sh*Math.toRadians(270));
        parkWall = new Pose2d(sx*68, 44.5, flip + sh*Math.toRadians(270));
        teamElementLeft = new Pose2d(sx*28, 8, flip + sh*Math.toRadians(90));
        teamElementRight = new Pose2d(sx*36, 23, flip + sh*Math.toRadians(20));
        teamElementCenter = new Pose2d(sx* 27.5, 15, flip + sh*Math.toRadians(0));
        teamElementLeftLow= new Pose2d(sx*32, -35, flip + sh*Math.toRadians(50));
        teamElementRightLow = new Pose2d(sx*30, -21, flip + sh*Math.toRadians(270));
        teamElementCenterLow = new Pose2d(sx*28, -29, flip + sh*Math.toRadians(0));
        extraPointLow = new Pose2d(sx*32, -30, flip + sh*Math.toRadians(270));
        extraPointHigh = new Pose2d(sx*36, 23, flip + sh*Math.toRadians(90));


        starting_point_top = new Pose2d(sx*58,18,flip + sh*Math.toRadians(180));
        staring_point = new Pose2d(sx*50,-31,flip + sh*Math.toRadians(180));
        team_element = new Pose2d(sx*30,-28,flip + sh*Math.toRadians(90));
        white_pixel_middle = new Pose2d(sx*22,-62,flip + sh*Math.toRadians(270));
        center = new Pose2d(sx*6,3,flip + sh*Math.toRadians(270));
        back_drop = new Pose2d(sx*30, 56,flip + sh*Math.toRadians(270));
        red_side =new Pose2d(sx*58,3,flip + sh*Math.toRadians(270));

        /* Qualifier Route Points */
        if(RobotConstants.bot == RobotConstants.Chassis.B7252) {
            moveAwayFromWallRedBackdrop = new Pose2d(50, 15, Math.toRadians(180));
            moveAwayFromWallBlueBackdrop = new Pose2d(-50, 15, Math.toRadians(0));

            moveAwayFromWallRedStacks = new Pose2d(52, -35, Math.toRadians(180));
            moveAwayFromWallBlueStacks = new Pose2d(-54, -35, Math.toRadians(0));

            moveTowardsRedBackdrop = new Pose2d(64.5, 20, Math.toRadians(270));
            moveTowardsBlueBackdrop = new Pose2d(-60, 8, Math.toRadians(270));

            moveTowardsRedBackdropLft = new Pose2d(59, 15, Math.toRadians(270));

            moveTowardsBlueBackdropHdAdj = new Pose2d(-35, 38, Math.toRadians(245));
            moveTowardsRedBackdropHdAdj = new Pose2d(40, 38, Math.toRadians(295));

            moveTowardsRedBackdropHdAdjLft = new Pose2d(35, 30, Math.toRadians(295));
            moveTowardsRedBackdropHdAdjRt = new Pose2d(30, 30, Math.toRadians(295));

            dropPixelRedRightTapeBackdrop = new Pose2d(34, 24, Math.toRadians(0));
            dropPixelRedCenterTapeBackdrop = new Pose2d(32, 15, Math.toRadians(0));
            dropPixelRedLeftTapeBackdrop = new Pose2d(32, 5, Math.toRadians(90));

            dropPixelRedLeftTapeBackdropAdj = new Pose2d(32, 9, Math.toRadians(90));
            dropPixelRedRightTapeBackdropAdj = new Pose2d(36, 27, Math.toRadians(0));

            dropPixelBlueRightTapeBackdrop = new Pose2d(-33, 5, Math.toRadians(90));
            dropPixelBlueCenterTapeBackdrop = new Pose2d(-29, 15, Math.toRadians(180));
            dropPixelBlueLeftTapeBackdrop = new Pose2d(-33, 18, Math.toRadians(270));

            dropPixelBlueCenterBackTapeAdj = new Pose2d(-33, 15, Math.toRadians(-180));
            dropPixelRedCenterBackTapeAdj = new Pose2d(32, 18, Math.toRadians(0));

            dropPixelRedRightTapeStacks = new Pose2d(32, -30.5, Math.toRadians(270));
            dropPixelRedCenterTapeStacks = new Pose2d(21.5, -45, Math.toRadians(270));
            dropPixelRedLeftTapeStacks = new Pose2d(32, -46, Math.toRadians(0));

            dropPixelRedRightTapeStacksAdj = new Pose2d(32, -46, Math.toRadians(270));

            dropPixelRedCenterStkTapeAdj = new Pose2d(26, -45, Math.toRadians(270));
            dropPixelBlueCenterStkTapeAdj = new Pose2d(-27, -46, Math.toRadians(270));

            dropPixelBlueRightTapeStacks = new Pose2d(-32, -52, Math.toRadians(270));
            dropPixelBlueCenterTapeStacks = new Pose2d(-20, -42, Math.toRadians(270));
            dropPixelBlueLeftTapeStacks = new Pose2d(-33, -30, Math.toRadians(270));

            moveFromBlueRightTapeStacks = new Pose2d(-32, -57.5, Math.toRadians(270));
            moveFromBlueLeftTapeStacks = new Pose2d(-32, -44, Math.toRadians(270));
            moveFromBlueCenterTapeStacks = new Pose2d(-24, -52, Math.toRadians(270));

            moveFromRedLeftTapeStacks = new Pose2d(59, -50, Math.toRadians(270));
            moveFromRedRightTapeStacks = new Pose2d(32, -40, Math.toRadians(270));
            moveFromRedCenterTapeStacks = new Pose2d(24, -52, Math.toRadians(230));

            dropOnBackdropBlueLeftBackdrop = new Pose2d(-46.7, 52.5, Math.toRadians(270));
            dropOnBackdropBlueRightBackdrop = new Pose2d(-25, 53.5, Math.toRadians(270));
            dropOnBackdropBlueCenterBackdrop = new Pose2d(-37, 53.5, Math.toRadians(270));

            dropOnBackdropBlueLeftBackdropHi = new Pose2d(-43, 55.5, Math.toRadians(270));
            dropOnBackdropBlueRightBackdropHi = new Pose2d(-25, 55.5, Math.toRadians(270));
            dropOnBackdropBlueCenterBackdropHi = new Pose2d(-37, 55.5, Math.toRadians(270));

            dropOnBackdropRedLeftBackdrop = new Pose2d(28, 52, Math.toRadians(270));
            dropOnBackdropRedRightBackdrop = new Pose2d(45.5, 56.3, Math.toRadians(270));
            dropOnBackdropRedCenterBackdrop = new Pose2d(36.5, 55.5, Math.toRadians(270));

            moveAwayFromCRedBackdropTape = new Pose2d(38, 15, Math.toRadians(0));
            moveAwayFromLRedBackdropTape = new Pose2d(32, 15, Math.toRadians(90));
            moveAwayFromRRedBackdropTape = new Pose2d(41, 24, Math.toRadians(0));

            moveAwayFromCBlueBackdropTape = new Pose2d(-45, 15, Math.toRadians(180));
            moveAwayFromLBlueBackdropTapeAdj = new Pose2d(-33, 14, Math.toRadians(270));
            moveAwayFromRBlueBackdropTape = new Pose2d(-32, 15, Math.toRadians(90));

            moveAwayFromLBlueBackdropTape = new Pose2d(-50, 14, Math.toRadians(270));

            reverseFromRedBackdropBk = new Pose2d(36, 35, Math.toRadians(270));
            reverseFromBlueBackdropBk = new Pose2d(-36, 42, Math.toRadians(270));

            reverseFromRedBackdropStk = new Pose2d(36, 42, Math.toRadians(310));
            reverseFromBlueBackdropStk = new Pose2d(-36, 42, Math.toRadians(230));

            dropOnBackdropBlueLeftStacks = new Pose2d(-39, 55.5, Math.toRadians(270));
            dropOnBackdropBlueRightStacks = new Pose2d(-16.5, 59, Math.toRadians(270));
            dropOnBackdropBlueCenterStacks = new Pose2d(-31.5, 57, Math.toRadians(270));

            dropOnBackdropBlueLeftStacksHi = new Pose2d(-39, 57, Math.toRadians(270));
            dropOnBackdropBlueRightStacksHi = new Pose2d(-16.5, 60, Math.toRadians(270));
            dropOnBackdropBlueCenterStacksHi = new Pose2d(-31.5, 58.5, Math.toRadians(270));
            backupFromBackdropBlueStacksCenter = new Pose2d(-31.5, 46.5, Math.toRadians(270));

            dropOnBackdropRedLeftStacks = new Pose2d(27.5, 53.5, Math.toRadians(270));
            dropOnBackdropRedRightStacks = new Pose2d(51, 61, Math.toRadians(270));
            dropOnBackdropRedCenterStacks = new Pose2d(42.5, 50.5, Math.toRadians(270));

            dropOnBackdropRedLeftStacksHi = new Pose2d(27.5, 54.5, Math.toRadians(270));
            dropOnBackdropRedRightStacksHi = new Pose2d(51, 62.5, Math.toRadians(270));
            dropOnBackdropRedCenterStacksHi = new Pose2d(44, 52, Math.toRadians(270));

            backAwayFromRedTape = new Pose2d(64.5, -36, Math.toRadians(270));
            backAwayFromBlueTape = new Pose2d(-60, -45, Math.toRadians(270));

            parkBlueBackdropSide = new Pose2d(-64, 46, Math.toRadians(270));
            parkRedBackdropSide = new Pose2d(64, 52, Math.toRadians(245));

            parkBlueStacksSide = new Pose2d(0, 55, Math.toRadians(230));
            parkRedStacksSide = new Pose2d(12, 53, Math.toRadians(270));

            parkRedStacksSideRt = new Pose2d(10, 50, Math.toRadians(270));
            parkRedStacksSideLft = new Pose2d(10, 50, Math.toRadians(270));
        } else if(RobotConstants.bot == RobotConstants.Chassis.B7253) {
        moveAwayFromWallRedBackdrop = new Pose2d(50, 15, Math.toRadians(180));
        moveAwayFromWallBlueBackdrop = new Pose2d(-50, 15, Math.toRadians(0));

        moveAwayFromWallRedStacks = new Pose2d(52, -35, Math.toRadians(180));
        moveAwayFromWallBlueStacks = new Pose2d(-54, -35, Math.toRadians(0));

        moveTowardsRedBackdrop = new Pose2d(62, 20, Math.toRadians(270));
        moveTowardsBlueBackdrop = new Pose2d(-60, 8, Math.toRadians(270));

        moveTowardsRedBackdropLft = new Pose2d(sx*59, 15, flip + sh*Math.toRadians(270));
        byBlueLoadStation = new Pose2d(sx * 59, -48, flip + sh*Math.toRadians(270));

        moveTowardsBlueBackdropHdAdj = new Pose2d(-35, 38, Math.toRadians(245));
        moveTowardsRedBackdropHdAdj = new Pose2d(40, 45, Math.toRadians(295));

        moveTowardsRedBackdropHdAdjLft = new Pose2d(35, 30, Math.toRadians(295));
        moveTowardsRedBackdropHdAdjRt = new Pose2d(30, 45, Math.toRadians(295));

        dropPixelRedRightTapeBackdrop = new Pose2d(sx*34, 20.5, flip + sh*Math.toRadians(-30));
        dropPixelRedCenterTapeBackdrop = new Pose2d(sx*28, 16, flip + sh*Math.toRadians(-1));
        dropPixelRedLeftTapeBackdrop = new Pose2d(sx * 31, 3, flip + sh*Math.toRadians(90));

        dropPixelRedLeftTapeBackdropAdj = new Pose2d(32, 9, Math.toRadians(90));
        dropPixelRedRightTapeBackdropAdj = new Pose2d(36, 27, Math.toRadians(0));

        dropPixelBlueRightTapeBackdrop = new Pose2d(-31, 4, Math.toRadians(90));
        dropPixelBlueCenterTapeBackdrop = new Pose2d(-28, 15, Math.toRadians(180));
        dropPixelBlueLeftTapeBackdrop = new Pose2d(-31, 18, Math.toRadians(270));
            dropPixelBlueLeftTapeBackdrop2 = new Pose2d(-38, 22, Math.toRadians(180));

        dropPixelBlueCenterBackTapeAdj = new Pose2d(-33, 15, Math.toRadians(-180));
        dropPixelRedCenterBackTapeAdj = new Pose2d(32, 18, Math.toRadians(0));

        dropPixelRedRightTapeStacks = new Pose2d(28, -26, Math.toRadians(270));
        dropPixelRedCenterTapeStacks = new Pose2d(18.5, -40, Math.toRadians(270));
//        dropPixelRedLeftTapeStacks = new Pose2d(32, -45, Math.toRadians(0));
        dropPixelRedLeftTapeStacks = new Pose2d(26, -49, Math.toRadians(270));

        dropPixelRedRightTapeStacksAdj = new Pose2d(32, -46, Math.toRadians(270));

        dropPixelRedCenterStkTapeAdj = new Pose2d(26, -45, Math.toRadians(270));
        dropPixelBlueCenterStkTapeAdj = new Pose2d(-27, -46, Math.toRadians(270));

        dropPixelBlueRightTapeStacks = new Pose2d(-26, -49, Math.toRadians(270));
        dropPixelBlueCenterTapeStacks = new Pose2d(-19.5, -41, Math.toRadians(270));
        dropPixelBlueLeftTapeStacks = new Pose2d(-28.5, -26.75, Math.toRadians(270));

        moveFromBlueRightTapeStacks = new Pose2d(-35, -57, Math.toRadians(270));
        moveFromBlueLeftTapeStacks = new Pose2d(-32, -44, Math.toRadians(270));
        moveFromBlueCenterTapeStacks = new Pose2d(-24, -52, Math.toRadians(270));

        moveFromRedLeftTapeStacks = new Pose2d(33.5, -58, Math.toRadians(270));
        moveFromRedRightTapeStacks = new Pose2d(32, -40, Math.toRadians(270));
        moveFromRedCenterTapeStacks = new Pose2d(24, -52, Math.toRadians(230));

        dropOnBackdropBlueLeftBackdropHi = new Pose2d(-43, 55.5, Math.toRadians(270));
        dropOnBackdropBlueRightBackdropHi = new Pose2d(-25, 55.5, Math.toRadians(270));
        dropOnBackdropBlueCenterBackdropHi = new Pose2d(-37, 55.5, Math.toRadians(270));

        dropOnBackdropRedLeftBackdrop = new Pose2d(sx*24, 56, flip + sh*Math.toRadians(270 -.01));
        dropOnBackdropRedRightBackdrop = new Pose2d(sx*41, 64, flip + sh*Math.toRadians(270));
        dropOnBackdropRedCenterBackdrop = new Pose2d(sx*30.25, 63.5, flip + sh*Math.toRadians(270));

        dropOnBackdropBlueRightBackdrop = new Pose2d(sx*26.25, 54.75, flip + sh*Math.toRadians(269.99));
        dropOnBackdropBlueLeftBackdrop = new Pose2d(sx*39.5, 63, flip + sh*Math.toRadians(270));
        dropOnBackdropBlueCenterBackdrop = new Pose2d(sx*30.25, 60, flip + sh*Math.toRadians(270));

        moveAwayFromCRedBackdropTape = new Pose2d(38, 15, Math.toRadians(0));
        moveAwayFromLRedBackdropTape = new Pose2d(32, 15, Math.toRadians(90));
        moveAwayFromRRedBackdropTape = new Pose2d(41, 24, Math.toRadians(0));

        moveAwayFromCBlueBackdropTape = new Pose2d(-45, 15, Math.toRadians(180));
        moveAwayFromLBlueBackdropTapeAdj = new Pose2d(-33, 14, Math.toRadians(270));
        moveAwayFromRBlueBackdropTape = new Pose2d(-32, 15, Math.toRadians(90));

        moveAwayFromLBlueBackdropTape = new Pose2d(-49, 14, Math.toRadians(270));

        reverseFromRedBackdropBk = new Pose2d(36, 38, Math.toRadians(270));
        reverseFromBlueBackdropBk = new Pose2d(-36, 42, Math.toRadians(270));

        reverseFromRedBackdropStk = new Pose2d(36, 42, Math.toRadians(310));
        reverseFromBlueBackdropStk = new Pose2d(-36, 42, Math.toRadians(230));

        dropOnBackdropBlueLeftStacks = new Pose2d(-42.5, 54.75, Math.toRadians(270));
        dropOnBackdropBlueRightStacks = new Pose2d(-32, 50.5, Math.toRadians(270));
        dropOnBackdropBlueCenterStacks = new Pose2d(-36, 54, Math.toRadians(270));

        dropOnBackdropBlueLeftStacksHi = new Pose2d(-42, 59, Math.toRadians(270));
        dropOnBackdropBlueRightStacksHi = new Pose2d(-27.5, 51.5, Math.toRadians(270));
        dropOnBackdropBlueCenterStacksHi = new Pose2d(-40.75, 58.5, Math.toRadians(270));
        backupFromBackdropBlueStacksCenter = new Pose2d(-31.5, 46.5, Math.toRadians(270));

        dropOnBackdropRedLeftStacks = new Pose2d(sx*30, 53.5, flip + sh*Math.toRadians(270));
        dropOnBackdropRedRightStacks = new Pose2d(46, 54, Math.toRadians(270));
        dropOnBackdropRedCenterStacks = new Pose2d(37, 48.5, Math.toRadians(270));

        dropOnBackdropRedLeftStacksHi = new Pose2d(29, 56, Math.toRadians(270));
        dropOnBackdropRedRightStacksHi = new Pose2d(41.5, 60, Math.toRadians(270));
        dropOnBackdropRedCenterStacksHi = new Pose2d(39, 56.5, Math.toRadians(270));

        backAwayFromRedTape = new Pose2d(64.5, -36, Math.toRadians(270));
        backAwayFromBlueTape = new Pose2d(-64, -37.5, Math.toRadians(270));

        parkBlueBackdropSide = new Pose2d(-64, 46, Math.toRadians(270));
        parkRedBackdropSide = new Pose2d(64, 52, Math.toRadians(245));

        parkBlueStacksSide = new Pose2d(0, 55, Math.toRadians(230));
        parkRedStacksSide = new Pose2d(12, 53, Math.toRadians(270));

        parkRedStacksSideRt = new Pose2d(10, 50, Math.toRadians(270));
        parkRedStacksSideLft = new Pose2d(10, 50, Math.toRadians(270));
    }
        /* End Qualifier Route Points */

        //2023 tangents
        ninetyFive = flip + sh*Math.toRadians(95);
        ninety = flip + sh*Math.toRadians(90);
        seventy = flip + sh*Math.toRadians(70);
        sixty = flip + sh*Math.toRadians(60);
        oneFifteen = flip + sh*Math.toRadians(115);
        twotwentyfive = flip + sh*Math.toRadians(225);
        twotwenty = flip + sh*Math.toRadians(220);
        twosixty = flip + sh*Math.toRadians(260);
        oneten = flip + sh*Math.toRadians(110);
        twofifity = flip + sh*Math.toRadians(250);
        oneeighty = flip + sh*Math.toRadians(180);
        thirtyfive = flip + sh*Math.toRadians(35);
        thirty = flip + sh*Math.toRadians(30);
        ten = flip + sh*Math.toRadians(10);
        oneHundred = flip + sh*Math.toRadians(100);
        twoTen = flip + sh*Math.toRadians(210);
        forty = flip + sh*Math.toRadians(40);
        zero = flip + sh*Math.toRadians(0);
        onesixtyfive = flip + sh*Math.toRadians(165);
        fifteen = flip + sh*Math.toRadians(15);
        threetwenty = flip + sh*Math.toRadians(320);
        oneNinety = flip + sh*Math.toRadians(190);
        eightyfive = flip + sh*Math.toRadians(85);
        oneforty = flip + sh*Math.toRadians(140);
        threeFifty = flip + sh*Math.toRadians(350);
        sixtyfive = flip + sh*Math.toRadians(65);
        onefifity = flip + sh*Math.toRadians(150);
        twoseventy = flip + sh*Math.toRadians(270);
        twoninety = flip + sh*Math.toRadians(290);
        twoeighty = flip + sh*Math.toRadians(280);
        fiftyfive = flip + sh*Math.toRadians(55);
        threefortyfive = flip + sh*Math.toRadians(345);

         if (INIT_TRAJ_2 != RobotConstants.trajType)
         {
             initTrajectories();
         }
         if (INIT_TRAJ_1 != RobotConstants.trajType)
         {
             initTrajectories2();
         }

     }


     protected void initTrajectories2()
     {
         finalizeTrajSeq();
     }

     protected void initTrajectories()
     {

     }


//	 public void elvToConeStack()
//     {
//         switch (conestackNum)
//         {
//              case 1:
//                  liftElvConeStackLvlOne();
//                  break;
//              case 2:
//                  liftElvConeStackLvlTwo();
//                  break;
//              case 3:
//                  liftElvConeStackLvlThree();
//                  break;
//              case 4:
//                  liftElvConeStackLvlFour();
//                  break;
//              case 5:
//                  liftElvConeStackLvlFive();
//                  break;
//              default:
//                  liftElvConeStackLvlOne();
//                  break;
//         }
//         conestackNum --;
//     }
//
//    protected void liftElvGndJnct()
//    {
//        if (robot.elev != null)
//        {
//            robot.elev.moveToLevel(0, RobotConstants.EL_SPD);
//        }
//
//    }
//
//     protected void liftElvLowPoleJnct()
//     {
//         if (robot.elev != null)
//         {
//             RobotLog.dd(TAG,"Moving Lifter to %.2f",
//                         RobotConstants.LF_EXT_LEVS[0] + 1.0);
//             RobotLog.dd(TAG, robot.elev.toString());
//
//             robot.elev.moveToLevel(1, RobotConstants.EL_SPD);
//         }
//     }
//
//     protected void liftElvMediumPoleJnct()
//     {
//         if(robot.elev != null)
//         {
//             RobotLog.dd(TAG,"Moving elev at +rate");
//             RobotLog.dd(TAG, robot.elev.toString());
//             robot.elev.moveToLevel(2, RobotConstants.EL_SPD);
//         }
//
//     }
//
//     /* Lift Elevator to the Highest Level to prepare to drop Cone */
//     protected void liftElvHighPoleJnct()
//     {
//         if(robot.elev != null)
//         {
//             RobotLog.dd(TAG,"Moving armExtend to level 1");
//             RobotLog.dd(TAG, robot.elev.toString());
//             robot.elev.moveToLevel(3, RobotConstants.EL_SPD);
//         }
//     }
//
//     /* Lift Elevator to the Highest Level to prepare to drop Cone */
//     protected void liftElvConeStackLvlFive()
//     {
//         if(robot.elev != null)
//         {
//             RobotLog.dd(TAG,"Moving elevator to cone stack level 5");
//             RobotLog.dd(TAG, robot.elev.toString());
//             robot.elev.moveToLevel(4, RobotConstants.EL_SPD);
//         }
//     }
//
//     protected void liftElvConeStackLvlFour()
//     {
//         if(robot.elev != null)
//         {
//             RobotLog.dd(TAG,"Moving elevator to cone stack level 4");
//             RobotLog.dd(TAG, robot.elev.toString());
//             robot.elev.moveToLevel(5, RobotConstants.EL_SPD);
//         }
//     }
//
//     protected void liftElvConeStackLvlThree()
//     {
//         if(robot.elev != null)
//         {
//             RobotLog.dd(TAG,"Moving elevator to cone stack level 3");
//             RobotLog.dd(TAG, robot.elev.toString());
//             robot.elev.moveToLevel(6, RobotConstants.EL_SPD);
//         }
//     }
//
//     protected void liftElvConeStackLvlTwo()
//     {
//         if(robot.elev != null)
//         {
//             RobotLog.dd(TAG,"Moving elevator to cone stack level 2");
//             RobotLog.dd(TAG, robot.elev.toString());
//             robot.elev.moveToLevel(7, RobotConstants.EL_SPD);
//         }
//     }
//
//     protected void liftElvConeStackLvlOne()
//     {
//         if(robot.elev != null)
//         {
//             RobotLog.dd(TAG,"Moving elevator to cone stack level 1");
//             RobotLog.dd(TAG, robot.elev.toString());
//             robot.elev.moveToLevel(0, RobotConstants.EL_SPD);
//         }
//     }
//
    protected void setClawPos(double pos){
         
     }


     public void openclaw (){

         if(robot.claw != null)
         {
             robot.claw.openClaw(1);

         }


     }
//     protected void elvConeStackSecondOption()
//	 {
//         //setClawPos(.6);
//         liftElvGndJnct();
//         //setClawPos(.9);
//         liftElvMediumPoleJnct();
//    }
//
//     /* Release grasped Cone */
//     protected void openClaw()
//     {
////         if(robot.claw != null)
////         {
////             robot.claw.openClaw();
////         }
//     }
//
public void slidesUpOne(){
    RobotLog.dd(TAG, "set slides pos to 4");
    robot.slides.moveToLevel(4);
}



public void moveToPosition4(){
    slidesUpOne();
    armUpLittle();
}


    public void moveArmToBack(){
        RobotLog.dd(TAG, "moveArmToBack");
        robot.arm.moveToLevel(6, 1);
        robot.slides.moveToLevel(3);
    }

     protected void moveArmForward(){
         RobotLog.dd(TAG, "MOVE ARM FORWARD");
         robot.arm.moveToLevel(0,1);

     }

     protected void moveArmBackward(){
         RobotLog.dd(TAG, "MOVE Arm Back");
         robot.arm.moveToLevel(2,1);

     }

    public void armUpLittle(){
        RobotLog.dd(TAG, "Arm up pos little");
        robot.arm.moveToLevel(4, 1);

    }

    private Timer mTimer1;
    private TimerTask mTt1;
    private Timer mTimer2;
    private TimerTask mTt2;
    private Timer mTimer3;
    private TimerTask mTt3;


    public void deliverSample() {
        RobotLog.dd(TAG, "Arm up pos little");
        moveArmToDrop();


        mTimer1 = new Timer();
        mTt1 = new TimerTask() {
            public void run() {
                openclaw();
            }
        };
        mTimer1.schedule(mTt1, 300);

        mTimer2 = new Timer();
        mTt2 = new TimerTask() {
            public void run() {
                moveArmTo90();
            }

        };
        mTimer2.schedule(mTt2, 700);

        mTimer3 = new Timer();
        mTt3 = new TimerTask() {
            public void run() {
                minSlides();
            }
        };
        mTimer3.schedule(mTt3, 850);
//        route.addFunction(route::moveArmToDrop );
//        route.addEvent(Route.Action.WAIT,0.2);
//        route.addFunction(route::openclaw);
//        route.addEvent(Route.Action.WAIT,0.3);
//        route.addFunction(route::moveArmTo90 );
//        route.addEvent(Route.Action.WAIT,0.2);
//        route.addFunction(route::minSlides);
  }
     protected void shootWiffleFar(){
         RobotLog.dd(TAG, "Shoot Balls Far from small triangle");
     }
    protected void shootFar(){
        robot.shooter.defaultFarShooterTraj();
        robot.shooter.spinShooterW(MAX_SHOOTER_DIST);
         RobotLog.dd(TAG, "Turn on wheel");
    }
    protected void wheelOff(){
        robot.shooter.stopWheel();
    }
     protected void shootWiffleClose(){
         robot.shooter.defaultCloseShooterTraj();
         robot.shooter.spinShooterW(CLOSE_SHOOTER_DIST);
        RobotLog.dd(TAG, "Shoot Balls Close");
     }
     protected void transitonUp(){
         RobotLog.dd(TAG, "Transition Up ALL");
     }

     protected void threeTransitionsDown(){
         robot.shooter.resetTransition();
     }
     protected void resetTrajAng(){
        robot.shooter.setShooterTrajPos(MIN_TRAJ_ENCODER);
     }
     protected void intakeOn(){
         robot.crAzYIntake.setPwr(1);
        RobotLog.dd(TAG, "IntakeOn");
     }
     protected void intakeOff (){
         robot.crAzYIntake.setPwr(0);
        RobotLog.dd(TAG, "Intake Off");
     }
     protected void doPark()
     {
         if(VERBOSE) { RobotLog.dd(TAG, "Parking bot"); }
     }
    protected void shoot1(){
        robot.shooter.shoot(RIGHT);
        RobotLog.dd(TAG, "Shoot Transition 1");}
    protected void shoot2(){
        robot.shooter.shoot(CENTER);
        RobotLog.dd(TAG, "Shoot Transition 2");}
    protected void shoot3(){
        robot.shooter.shoot(LEFT);
        RobotLog.dd(TAG, "Shoot Transition 3");}



    private double DRIVE_TO_SHOOT_AMT = 5.6;

    protected void shootMotif(int greenPos, Pose2d pos1){
        double theta = pos1.getHeading() - Math.toDegrees(90);
        Pose2d pos2 = new Pose2d(pos1.getX()+Math.cos(theta)*DRIVE_TO_SHOOT_AMT, pos1.getY()+Math.sin(theta)*DRIVE_TO_SHOOT_AMT, pos1.getHeading());
        Pose2d pos3 = new Pose2d(pos2.getX()+Math.cos(theta)*DRIVE_TO_SHOOT_AMT, pos2.getY()+Math.sin(theta)*DRIVE_TO_SHOOT_AMT, pos2.getHeading());
       switch (motif)
       {
           case SHOOTGPP:
               switch (greenPos){
                   case 1:  addLocation(pos1,LINE, Heading.HEAD_LINEAR);
                            addFunction(this::shoot1);
                            break;
                   case 2: addLocation(pos2,LINE, Heading.HEAD_LINEAR);
                            addFunction(this::shoot2);
                            break;
                   case 3: addLocation(pos3,LINE, Heading.HEAD_LINEAR);
                            addFunction(this::shoot3);
                            break;
               }
               addEvent(Route.Action.WAIT, 2.0);
               switch (greenPos){
                   case 1: addLocation(pos2,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot2);
                       addLocation(pos3,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot3);
                       break;
                   case 2:addLocation(pos1,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot1);
                       addLocation(pos3,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot3);
                       break;
                   case 3: addLocation(pos1,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot1);
                       addLocation(pos2,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot2);
                       break;
               }
               addEvent(Route.Action.WAIT, 0.34);
               break;
           case SHOOTPGP:
               switch (greenPos){
                   case 1: addLocation(pos2,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot2);
                       break;
                   case 2:
                   case 3: addLocation(pos1,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot1);
                       break;
               }
               addEvent(Route.Action.WAIT, 2.0);
               switch (greenPos){
                   case 1:
                       addLocation(pos1,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot1);
                       break;
                   case 2:
                       addLocation(pos2,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot2);
                       break;
                   case 3:
                       addLocation(pos3,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot3);
                       break;
               }
               addEvent(Route.Action.WAIT, 2.0);
               switch (greenPos){
                   case 1:
                   case 2:
                       addLocation(pos3,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot3);
                       break;
                   case 3:
                       addLocation(pos2,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot2);
                       break;
               }
               addEvent(Route.Action.WAIT, 0.34);

               break;
           case SHOOTPPG:
               switch (greenPos){
                   case 1:
                       addLocation(pos2,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot2);
                   addEvent(Route.Action.WAIT, 3.0);
                       addLocation(pos3,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot3);
                       break;
                   case 2:
                       addLocation(pos1,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot1);
                       addEvent(Route.Action.WAIT, 3.0);
                       addLocation(pos3,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot3);
                       break;
                   case 3:
                       addLocation(pos1,LINE, Heading.HEAD_LINEAR);

                       addFunction(this::shoot1);

                       addEvent(Route.Action.WAIT, 3.0);
                       addLocation(pos2,LINE, Heading.HEAD_LINEAR);

                       addFunction(this::shoot2);
                       break;
               }
               addEvent(Route.Action.WAIT, 3.0);
               switch (greenPos){
                   case 1:
                       addLocation(pos1,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot1);
                       break;
                   case 2:
                       addLocation(pos2,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot2);
                       break;

                   case 3:
                       addLocation(pos3,LINE, Heading.HEAD_LINEAR);
                       addFunction(this::shoot3);
                       break;
               }
               addEvent(Route.Action.WAIT, 1);
               break;
       }
     }


     public String toString()
     {
         return "";
     }

     public final static int INIT_CONE_STACK = 5;
     public static int conestackNum = INIT_CONE_STACK;
     protected MecanumBot robot;
	 protected Field.Park_Pos parkPos;
     protected PositionOption startPos;
     protected Field.Alliance alliance;
     protected Field.Route routeStrategy;
     protected Field.Wiffle_Pos lastLocation;
     protected Field.Motif motif;
     protected static Field.Num_shots numshot;
     public TeamElement teamElement;
     protected Field.Parks[] highways;
	 protected Field.Parks[] pixelStacks;
     protected int numCycles = 0;
     protected double botLen;
     protected double botWid;
     protected double botBackToCtr;
     protected double botSideToCtr;
     protected double hubRad;

     protected int sx;
     protected int sy;
     protected double sh;
     protected int sf;
     protected int sr;
     protected double flip;
     protected double strtX;
     protected double strtY;
     protected double strtH;

     protected Pose2d moveFromStart;
     protected Pose2d turnBox;
     protected Pose2d coneDropSECAJUST;
	 //public final Pose2d Start;
     protected Pose2d awayFromStart;
     protected Pose2d awayFromStart2;
     protected Pose2d toTurnBox;
     //protected final Pose2d seven;
     //protected final Pose2d six;
    protected Pose2d red_side;
    protected Pose2d center;
    protected Pose2d top;

   protected Pose2d middle;

    protected Pose2d bottom;
     protected Pose2d staring_point;
     protected Pose2d team_element;
     protected Pose2d white_pixel_middle;
     protected Pose2d back_drop;
     protected Pose2d startCSRedHigh;
     protected Pose2d startSpecimenSide;
     protected Pose2d specimen1;
     protected Pose2d specimen1PosPosToPush;
     protected Pose2d specimen1PosToPush;
     protected Pose2d dropPreSpecimen;
    protected Pose2d corner;
    protected Pose2d corner2;
    protected Pose2d subPos;
    protected Pose2d subPos2;
    protected Pose2d subPos3;
    protected Pose2d corner3;
    protected Pose2d preSubPos;
     protected Pose2d startSample;
     protected Pose2d startSmallTri;
    protected Pose2d simpleShootFar;

    protected Pose2d startSmallTriBlue;
    protected Pose2d shootFarPosBLUE;
    protected Pose2d PrepPosIntake1;
    protected Pose2d PrepPosIntake1BLUE;

    protected Pose2d intookpos1;
    protected Pose2d intookpos1BLUE;
    protected Pose2d intookpos1BLUEDONE;

    protected Pose2d intakepos2;
    protected Pose2d intakepos2B;
    protected Pose2d intakeloading2;
    protected Pose2d intakeloading2B;
    protected Pose2d intakepos3;
    protected Pose2d intakepos3B;
    protected Pose2d intakeloading3;
    protected Pose2d intakeloading3B;
    protected Pose2d shootPos3;
    protected Pose2d intakepos4;
    protected Pose2d intakeloading4;
    protected Pose2d shootNearPos;
    protected Pose2d shootNearPosB;

     protected Pose2d preleverwiffle;
     protected Pose2d intakewifflelever;
     protected Pose2d hitlever;
     protected Pose2d prenearpos;
     protected Pose2d nearpos;
     protected Pose2d shootFarPos;

    protected Pose2d shootFarPosB;
     protected Pose2d pregotoparkwiffle;
     protected Pose2d  gotoparkwiffle;
     protected Pose2d pregotogoalwiffles;
     protected Pose2d gotogoalwiffles;
     protected Pose2d parkInside1Red;
     protected Pose2d parkInside2Red;
     protected Pose2d parkOutside1Red;
     protected Pose2d parkOutside2Red;
     protected Pose2d parkInside1Blue;
     protected Pose2d parkInside2Blue;
     protected Pose2d parkOutside1Blue;
    protected Pose2d parkOutside2BLUE;
    protected Pose2d tempParkRed;
    protected Pose2d tempParkBlue;
    protected Pose2d startSpecimen;
     protected Pose2d purplePixelPlaceCenterTop;
    protected Pose2d dropCenterPixel;
    protected Pose2d hangSpecimen;
    protected Pose2d moveBackFromSpecimen;
    protected Pose2d sample1;
    protected Pose2d redWall;
    protected Pose2d blueFar;
    protected Pose2d shootPreWhiffles;
    protected Pose2d blueWall;
    protected Pose2d aroundPartner;
    protected Pose2d shootZonefornear;
    protected Pose2d shootZonefornearblue;

    protected Pose2d aroundPartnerBlue;
    protected Pose2d intakeGoal;
    protected Pose2d intakeGoalBlue;
    protected Pose2d intakeGoalWhiffles;
    protected Pose2d shootGoalWhiffles;
    protected Pose2d intakeGate;
    protected Pose2d intakeGateBlue;
    protected Pose2d intakeGateWhiffles;
    protected Pose2d intakeGateWhifflesBlue;
    protected Pose2d intakeGoalWhifflesBlue;
    protected Pose2d shootGateWhiffles;
    protected Pose2d shootGateWhifflesBlue;
    protected Pose2d aroundWhiffles;
    protected Pose2d shootGoalWhifflesBlue;
    protected Pose2d intakePark;
    protected Pose2d intakeParkBlue;
    protected Pose2d intakeParkWhiffles;
    protected Pose2d intakeParkWhifflesBlue;
    protected Pose2d shootParkWhiffles;
    protected Pose2d shootParkWhifflesBlue;
    protected Pose2d redGoal;
    protected Pose2d blueGoal;
    protected Pose2d sample2;
    protected Pose2d rotateToSample2;
    protected Pose2d sample3;
    protected Pose2d positionSample3;
    protected Pose2d turnStraight;
    protected Pose2d positionToPark;
    protected Pose2d park;
    protected Pose2d park2;

    protected Pose2d deliverSample3ToBasket;
    protected Pose2d deliverSample2ToBasket;
    protected Pose2d deliverSampleToBasket;
    protected Pose2d dropCenterPixelAdj;
    protected Pose2d dropCenterPixelStacksAdj;
    protected Pose2d dropCenterPixelLeftAdj;
    protected Pose2d dropCenterPixelCenterAdjBlue;
    protected Pose2d dropCenterPixelLeftAdjBlue;
    protected Pose2d dropCenterPixelRightAdjBlue;
    protected Pose2d dropCenterPixelLeftAdjRed;
    protected Pose2d dropCenterPixelBackwards;
    protected Pose2d dropLeftPixelBackwardsBlue;
    protected Pose2d dropRightPixelBackwardsRed;
    protected Pose2d dropRightPixelBackwardsBlue;
    protected Pose2d dropLeftPixelBackwardsRed;
    protected Pose2d dropCenterPixelBackwardsBlue;
    protected Pose2d dropCenterPixelBackwardsRed;
    protected Pose2d dropCenterPixelBackwardsRedBd;
    protected Pose2d dropRightPixelBackwardsRedBd;
    protected Pose2d dropLeftPixelBackwardsRedBd;
    protected Pose2d dropCenterPixelBackwardsLeftBackdrop;
    protected Pose2d whatchamacallit;
    protected Pose2d pickUpPixelStackLeft;
    protected Pose2d pickUpPixelStackLeftBlue;
    protected Pose2d pickUpPixelStackLeftLeft;
    protected Pose2d pickUpPixelStackLeftCenter;
    protected Pose2d pickUpPixelStackCenter;
    protected Pose2d pickUpPixelStackCenterStacks;
    protected Pose2d pickUpPixelStackCenterBlue;
    protected Pose2d pickUpPixelStackRight;
    protected Pose2d pickUpPixelStackRightCircuit;
    protected Pose2d pickUpPixelStackRightRedLeft;
    protected Pose2d pickUpPixelStackRightCircuitBlue;
    protected Pose2d pickUpPixelStackRightCircuitBlueCenter;
    protected Pose2d pickUpPixelStackRightCircuitBlueRight;
    protected Pose2d pickUpPixelStackRightLeft;
    protected Pose2d dropLeftPixel;
    protected Pose2d getDropRightPixelBlue;
    protected Pose2d startRedFar;
    protected Pose2d startRedWall;
    protected Pose2d shootfaronred;
    protected Pose2d startBlueFar;
    protected Pose2d moveToPark;
    protected Pose2d collect2;
    protected Pose2d nearLeaver;
    protected Pose2d collect3;
    protected Pose2d shootNear;
    protected Pose2d shootNearForSpline;
    protected Pose2d startBlueWall;
    protected Pose2d shootNearBlue;
    protected Pose2d goNear2Blue;
    protected Pose2d goNear3Blue;
    protected Pose2d goNear1Blue;
    protected Pose2d shootNearBlueSpline;
    protected Pose2d collect1Blue;
    protected Pose2d collect2Blue;
    protected Pose2d collect3Blue;
    protected Pose2d moveToHumanPlayerZone;
    protected Pose2d collect1;
    protected Pose2d shootNear2;

    protected Pose2d dropRightPixel;
    protected Pose2d dropRightPixelCircuit;
    protected Pose2d dropRightPixelCuircut;
    protected Pose2d dropLeftPixelBlue;
    protected Pose2d dropRightPixelBlue;
    protected Pose2d backAwayBackDrop;
    protected Pose2d underRigging;
    protected Pose2d pointForRobotToPixelStack;
    protected Pose2d pointForRobotToPixelStackBlue;
    protected Pose2d aboveRigging;
    protected Pose2d aboveRiggingBackwards;
    protected Pose2d underCenter;

    protected Pose2d pointcuzwhynot;
    protected Pose2d underdoor;
    protected Pose2d startCSRedLow;
    protected Pose2d parked;
    protected Pose2d startCBlueHigh;
    protected Pose2d startCSBlueLow;
    protected Pose2d underMiddlePoint;
    protected Pose2d parkDoor;
    protected Pose2d parkWall;
    protected Pose2d teamElementLeft;
    protected Pose2d teamElementRight;
    protected Pose2d teamElementCenter;
    protected Pose2d teamElementCenterLow;
    protected Pose2d teamElementLeftLow;
    protected Pose2d teamElementRightLow;

    protected Pose2d extraPointHigh;
    protected Pose2d extraPointLow;



    /* Qualifier Route Points */
    protected Pose2d moveAwayFromWallRedBackdrop;
    protected Pose2d moveAwayFromWallBlueBackdrop;

    protected Pose2d moveAwayFromWallRedStacks;
    protected Pose2d moveAwayFromWallBlueStacks;

    protected Pose2d moveTowardsRedBackdrop;
    protected Pose2d moveTowardsBlueBackdrop;

    protected Pose2d moveTowardsRedBackdropLft;
    protected Pose2d byBlueLoadStation;

    protected Pose2d moveTowardsBlueBackdropHdAdj;
    protected Pose2d moveTowardsRedBackdropHdAdj;

    protected Pose2d moveTowardsRedBackdropHdAdjLft;
    protected Pose2d moveTowardsRedBackdropHdAdjRt;

    protected Pose2d dropPixelRedRightTapeBackdrop;
    protected Pose2d dropPixelRedCenterTapeBackdrop;
    protected Pose2d dropPixelRedLeftTapeBackdrop;

    protected Pose2d dropPixelRedLeftTapeBackdropAdj;
    protected Pose2d dropPixelRedRightTapeBackdropAdj;

    protected Pose2d dropPixelBlueRightTapeBackdrop;
    protected Pose2d dropPixelBlueCenterTapeBackdrop;
    protected Pose2d dropPixelBlueLeftTapeBackdrop;
    protected Pose2d dropPixelBlueLeftTapeBackdrop2;

    protected Pose2d dropPixelRedRightTapeStacks;
    protected Pose2d dropPixelRedCenterTapeStacks;

    protected Pose2d dropPixelRedRightTapeStacksAdj;

    protected Pose2d dropPixelRedCenterStkTapeAdj;
    protected Pose2d dropPixelBlueCenterStkTapeAdj;

    protected Pose2d dropPixelRedCenterBackTapeAdj;
    protected Pose2d dropPixelRedLeftBackTapeAdj;
    protected Pose2d dropPixelBlueCenterBackTapeAdj;

    protected Pose2d dropPixelBlueCenterTapeStacks;
    protected Pose2d dropPixelBlueLeftTapeStacks;

    protected Pose2d moveFromBlueRightTapeStacks;
    protected Pose2d moveFromBlueLeftTapeStacks;
    protected Pose2d moveFromBlueCenterTapeStacks;

    protected Pose2d moveFromRedRightTapeStacks;
    protected Pose2d moveFromRedLeftTapeStacks;
    protected Pose2d moveFromRedCenterTapeStacks;

    protected Pose2d dropOnBackdropBlueLeftBackdrop;
    protected Pose2d dropOnBackdropBlueRightBackdrop;
    protected Pose2d dropOnBackdropBlueCenterBackdrop;

    protected Pose2d dropOnBackdropBlueLeftBackdropHi;
    protected Pose2d dropOnBackdropBlueRightBackdropHi;
    protected Pose2d dropOnBackdropBlueCenterBackdropHi;

    protected Pose2d dropOnBackdropRedLeftBackdrop;
    protected Pose2d dropOnBackdropRedRightBackdrop;
    protected Pose2d dropOnBackdropRedCenterBackdrop;

    protected Pose2d moveAwayFromCRedBackdropTape;
    protected Pose2d moveAwayFromLRedBackdropTape;
    protected Pose2d moveAwayFromRRedBackdropTape;

    protected Pose2d moveAwayFromCBlueBackdropTape;
    protected Pose2d moveAwayFromLBlueBackdropTapeAdj;
    protected Pose2d moveAwayFromRBlueBackdropTape;

    protected Pose2d moveAwayFromLBlueBackdropTape;

    protected Pose2d reverseFromRedBackdropBk;
    protected Pose2d reverseFromBlueBackdropBk;

    protected Pose2d reverseFromRedBackdropStk;
    protected Pose2d reverseFromBlueBackdropStk;

    protected Pose2d dropOnBackdropBlueLeftStacks;
    protected Pose2d dropOnBackdropBlueRightStacks;
    protected Pose2d dropOnBackdropBlueCenterStacks;

    protected Pose2d dropOnBackdropBlueLeftStacksHi;
    protected Pose2d dropOnBackdropBlueRightStacksHi;
    protected Pose2d dropOnBackdropBlueCenterStacksHi;
    protected Pose2d backupFromBackdropBlueStacksCenter;

    protected Pose2d dropOnBackdropRedLeftStacks;
    protected Pose2d dropOnBackdropRedRightStacks;
    protected Pose2d dropOnBackdropRedCenterStacks;

    protected Pose2d dropOnBackdropRedLeftStacksHi;
    protected Pose2d dropOnBackdropRedRightStacksHi;
    protected Pose2d dropOnBackdropRedCenterStacksHi;

    protected Pose2d backAwayFromRedTape;
    protected Pose2d backAwayFromBlueTape;

    protected Pose2d parkBlueBackdropSide;
    protected Pose2d parkRedBackdropSide;
    protected Pose2d parkBlueStacksSide;
    protected Pose2d parkRedStacksSide;

    protected Pose2d parkRedStacksSideLft;
    protected Pose2d parkRedStacksSideRt;

    protected Pose2d dropPixelRedLeftTapeStacks;
    protected Pose2d dropPixelBlueRightTapeStacks;

    /* End Qualifier Route Points */




     /* States Competition Points */
     public Pose2d start;
     protected Pose2d moveAwayFromStart;
     public Pose2d ajustForWheelHittingWall;
     protected Pose2d coneDropLowJunct;

    protected Pose2d start2;

    protected Pose2d specimen2;

    protected Pose2d specimen3;

    protected Pose2d specimenPark1;
    protected Pose2d waitForHumanPlayer;
    protected Pose2d waitForHumanPlayer2;
    protected Pose2d waitForHumanPlayer3;



    protected Pose2d forward;
     protected Pose2d fourPtJunctDropSpline;
     protected Pose2d fourPtJunctDropSplineAJUSTED;



     /* Parks */
     protected Pose2d parkCENTER;
     protected Pose2d parkRIGHT;
     protected Pose2d parkLEFT;

  protected Pose2d starting_point_top;

     /* Tangents */
     protected double moveAwayFromStartTangent;
     protected double fourPtJnctDropTangent;
     protected double reverseFourPtJunctDropTangent;
     protected double sharedHighJunctionDropSplineTangent;

     protected double sharedHighJunctionDropReverseToConstackTangent;
     protected double reverseHighJunctionDropSplineTangent;
     protected double oppositionHighJunctionDropSplineTangent;
     protected double reverseOppositionhighJunctionDropSplineTangent;
     protected double thelowtanjentreverse;
    protected double ninetyFive;
    protected double ninety;
    protected double seventy;
    protected double sixty;
    protected double oneFifteen;
    protected double twofifity;
    protected double oneeighty;
    protected double thirtyfive;
    protected double twotwentyfive;
    protected double twotwenty;
    protected double twosixty;
    protected double oneten;
    protected double oneHundred;
    protected double thirty;
    protected double ten;
    protected double twoTen;
    protected double forty;
    protected double oneforty;
    protected double zero;
    protected double onesixtyfive;
    protected double fifteen;
    protected double threetwenty;
    protected double eightyfive;
    protected double oneNinety;
    protected double threeFifty;
    protected double sixtyfive;
    protected double fiftyfive;
    protected double threefortyfive;
    protected double onefifity;
    protected double twoseventy;
    protected double twoninety;
    protected double twoeighty;

     protected double sharedHighJunctionDropSplineHEADING;
     protected double coneStackGrabHFromSharedHighJunctionHEADING;

     final static int MAX_SEGMENTS = 32;

	 /* Map for new route, leaves old route in tact in case we need to revert back to it to validate */
     public EnumMap<SegmentState, TrajectorySequence>  segmentStateMap = new EnumMap<>(Route.SegmentState.class);
     TrajectorySequence fullSeq;

     /* New way to define map */
     public List<TrajectorySequence> trajList;
     public EnumMap<ITD_Detector.Position,TrajectorySequence> parkMap = new EnumMap<>(ITD_Detector.Position.class);
     private Pose2d lastPose;
     private Pose2d firstPose;
     private TrajectorySequenceBuilder traj;
     protected double totalDur;

    public Pose2d getEnd(){
        if(null!= trajList && trajList.size() != 0){
            return trajList.get(trajList.size()-1).end();
        }
        else { return null; }
    }

     public enum Movement
     {
          START, //used for first position, i.e. no moving to this location
          LINE,
          STRAFE,
          STRAFE_LEFT,
          STRAFE_RIGHT,
          SPLINE,
          FORWARD,
          BACK,
          TURN
     }
     public enum Heading
     {
          HEAD_DEFAULT,
          HEAD_CONSTANT,
          HEAD_LINEAR,
          HEAD_SPLINE
     }

     public enum Action
     {
          WAIT,
          TANGENT
     }

     public enum TeamElement
     {
         LEFT,
         RIGHT,
         CENTER
     }

     public Pose2d getLastPose(){
        return lastPose;
     }

   public double calcTimeOrSomething(){
       double cumTime = 0.0;

       if(null != trajList)
       {


           int seqNum = 0;



           for (TrajectorySequence tSeq : trajList)
           {
               String seqName = String.valueOf(seqNum++);
               for (int i = 0; i < tSeq.size(); i++)
               {
                   SequenceSegment seg = tSeq.get(i);

                   if (seg instanceof TrajectorySegment)
                   {
                       Trajectory t = ((TrajectorySegment) seg).getTrajectory();
                       cumTime += t.duration();
                   }
                   else
                   {
                       if (seg instanceof TurnSegment)
                       {
                           TurnSegment ts = (TurnSegment) seg;
                           cumTime += ts.getDuration();
                       }
                       else if (seg instanceof WaitSegment)
                       {
                           WaitSegment ws = (WaitSegment) seg;
                           cumTime += ws.getDuration();
                       }
                   }
               }
           }

       }
       return cumTime;
    }



    public TrajectorySequence getTrajSeq(int trajNum){
         TrajectorySequence nextTraj = null;
         switch (trajNum){
             case 0:
                 nextTraj = fullSeq;
         }

         return nextTraj;
    }

     boolean justStarted = true;

     public void addLocation(Pose2d loc, Movement move, Heading head)
     {
         addLocation(loc, move, head, false, 0);
     }

    public void addLocation(Pose2d loc, Movement move, Heading head, double customHeadValue)
    {
        addLocation(loc, move, head, true, customHeadValue);
    }

    private boolean newTraj = false;

    public void makeNewTraj()
    {
        try {
            if (null != traj) {
                trajList.add(traj.build());
            }
            traj = new TrajectorySequenceBuilder(
                    lastPose,
                    defVelLim, defAccelLim,
                    RobotConstants.MAX_ANG_VEL, RobotConstants.MAX_ANG_ACCEL);
        }catch(Exception e){
			RobotLog.ww(TAG, "Error in makeNewTraj...not sure why.");
    }
    }

    public void addParkPosition(ITD_Detector.Position parkEnum, Pose2d loc, Movement move, Heading head)
    {
        Pose2d origLastPose = lastPose;
        addLocation(loc, move, head, false, 0);
        parkMap.put(parkEnum, traj.build());
        try
  	    {
            RobotLog.dd(TAG, "putting park position, %d", parkEnum);
        }
  	    catch (Exception e)
  	    {
              RobotLog.dd(TAG, "failure putting park position in log");
        }
        traj = null;
        lastPose = origLastPose;
    }

    private boolean samePose(Pose2d loc){
        boolean sp = false;
        if(loc.getY() == lastPose.getY() && loc.getX() == lastPose.getX()){
            sp = true;
            if(loc.getHeading() != lastPose.getHeading()){
                //TODO: throw compiler warning
            }
        }
        return sp;
    }

    public void addLocation(Pose2d loc, Movement move, Heading head, boolean custom, double customHeadValue)
    {
        RobotLog.dd(TAG, "adding Location: x:%f, y:%f, h:%f, move:%s,head:%s", loc.getX(), loc.getY(), Math.toDegrees(loc.getHeading()), move, head);
        if(null != lastPose)
  	    {
              if(!samePose(loc)) {
             if(justStarted)
  	         {
                 justStarted = false;
                 trajList = new ArrayList<TrajectorySequence>();
             }
             if(newTraj)
  	         {
	           makeNewTraj();  newTraj = false;
	         }
             else if(autonDebug == Field.AutonDebug.ENABLE)
	         {
	             //when in debug mode, default to a new Traj every time unless something set it to false
	             newTraj = true;
	         }          
             else
	         {
	             newTraj = false;
		         //when not in debug, default to false unless something set it to true;
	         }                                  

             switch(move)
  	         {
                  case START:
                      //acts as a restart; should not normally be used
                      firstPose = loc;
                      justStarted = true;
                      traj = new TrajectorySequenceBuilder(
                              firstPose,
                              defVelLim, defAccelLim,
                              RobotConstants.MAX_ANG_VEL, RobotConstants.MAX_ANG_ACCEL);
                      break;
                  case LINE:
                      switch (head)
                      {
                         case HEAD_LINEAR:
                           traj.lineToLinearHeading(loc);
                           break;
                         case HEAD_CONSTANT:
                           traj.lineToConstantHeading(loc.vec());
                           break;
                         case HEAD_SPLINE:
                           traj.lineToSplineHeading(loc);
                           break;
                         case HEAD_DEFAULT:
                         default:
                           traj.lineTo(loc.vec());
                           break;
                      }
                      break;
                  case STRAFE:
                      traj.strafeTo(loc.vec());
                      break;
                  default: //Spline or anything else
                      double headValue;

                      if(custom)
                      {
                          headValue = customHeadValue;
                      }
                      else
                      {
                          headValue = loc.getHeading();
                      }
                      switch (head)
                      {
                            case HEAD_LINEAR:
                              traj.splineToLinearHeading(loc,headValue);
                              break;
                            case HEAD_SPLINE:
                              traj.splineToSplineHeading(loc,headValue + flip);
                              break;
                            case HEAD_CONSTANT:
                              traj.splineToConstantHeading(loc.vec(),headValue + flip);
                              break;
                            case HEAD_DEFAULT:
                            default:
                              traj.splineTo(loc.vec(),headValue);
                              break;
                      }
                      break;
                  }
             }
        }
        else
  	    {
  	        //What should happen when first called with START
            firstPose = loc;
            justStarted = true;
            traj = new TrajectorySequenceBuilder(
                    firstPose,
                    defVelLim, defAccelLim,
                    RobotConstants.MAX_ANG_VEL, RobotConstants.MAX_ANG_ACCEL);
        }

        lastPose = loc; //Setup for next time

     }

     public void addMovement(Movement move, Double amount)
     {
          switch (move)
  	      {
               case STRAFE_RIGHT:
                 traj.strafeRight(amount);
                 break;
               case STRAFE_LEFT:
                 traj.strafeLeft(amount);
                 break;
               case FORWARD:
                 traj.forward(amount);
                 break;
               case BACK:
                 traj.back(amount);
                 break;
               case TURN:
                 traj.turn(amount);
                 break;
          }
     }

//     public void slowToBd(){
//
//     }

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static int DESIRED_TAG_ID = 2;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    double  turnA            = 0;        // Desired turning power/speed (-1 to +1)

    public void aprilDriveToBackdrop(){
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id%3 == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                }
            } else {

            }
        }

    }

     public void armDrivePos(){
        robot.arm.moveToLevel(1,1);
     }
     public void armDropSpikePos(){
         robot.arm.moveToCnt(-800,1);

}
    public void outFrontPixel(){

    }

 public void moveToDrive(){
        RobotLog.dd(TAG, "in moveToDrive\n");
     robot.arm.moveToLevel(2, 1);
     robot.slides.setLiftPos(RobotConstants.EL_MIN_ENCODER);
     }



    private Timer mTimerArm;
    private TimerTask mTtArm;

    public void delayedArmToPickup(){
        RobotLog.dd(TAG, "in delayedArmToPickup\n");

        mTimerArm = new Timer();
        mTtArm = new TimerTask() {
            public void run() {
                robot.arm.moveToLevel(0, 1);
                robot.slides.moveToLevel(0);
            }
        };
        mTimerArm.schedule(mTtArm, 1000);
    }

    public void moveArmToZero(){
        RobotLog.dd(TAG, "in moveArmToZero\n");
        robot.arm.moveToCnt(0,1);
    }

    public void moveArmToPickup(){
        RobotLog.dd(TAG, "in moveArmToPickup\n");
        robot.arm.moveToLevel(0, 1);
        robot.slides.moveToLevel(0);
    }

    public void moveSlidesToPickup(){
        RobotLog.dd(TAG, "in moveArmToPickup\n");
        robot.slides.setLiftPos(RobotConstants.EL_MIN_ENCODER);
    }

    public void moveArmTo90(){
        RobotLog.dd(TAG, "in moveArmTo90\n");
        robot.arm.moveToLevel(2, 1);
    }
    public void maxSlides(){
        RobotLog.dd(TAG, "in maxSlides\n");
        robot.slides.setLiftPos(RobotConstants.EL_LEV5);
    }
    public void moveArmToDrop(){
        RobotLog.dd(TAG, "in moveArmToDrop\n");
        robot.arm.moveToLevel(5, 1);
    }
    public void minSlides(){
        RobotLog.dd(TAG,  "in minSlides\n");
        robot.slides.setLiftPos(RobotConstants.EL_MIN_ENCODER);
    }


    public void armToDrop(){
        RobotLog.dd(TAG, "in armToDrop\n");
    }
    public void armToDropMid(){

    }

    public void armToDropHigher(){

    }
    public void armToIntake(){

    }
     public void outPixel(){
         if(VERBOSE) { RobotLog.dd(TAG, "in outPixel");}

     }
    public void outPurplePixel(){
        robot.purplePixelDrop = true;

    }

    public void intakes(){

    }
     public void intakesSlow(){

     }

     public void addFunction(MarkerCallback callback)
     {
          traj.addTemporalMarker(callback);
     }

     public void addFunction(MarkerCallback callback, double time)
     {
          traj.addTemporalMarker(time, callback);
     }

     public void addEvent(Action act, double value)
     {
          switch(act)
  	      {
              case WAIT:
                  traj.waitSeconds(value);
                  break;
              case TANGENT:
                  try{          //cath error when trying to add tangent to first trajectory. TODO: make this not needed
				      makeNewTraj();
			      }catch(Exception e){

                  }
                  newTraj = false;
                  traj.setTangent(value);
                  break;
          }
     }

    public void addTrajectory(TrajectorySequence trajectory)
    {
         if(null == trajList)
  	     {
             addLocation(trajectory.start(), Movement.START, Heading.HEAD_DEFAULT);
         }
         if(lastPose != trajectory.start())
  	     {
             addLocation(trajectory.start(), Movement.SPLINE, Heading.HEAD_SPLINE);
         }
         trajList.add(trajectory);
         lastPose = trajectory.end();
         newTraj = true;
    }

    public void addParkTrajectory(ITD_Detector.Position parkEnum, TrajectorySequence trajectory)
    {
         parkMap.put(parkEnum, trajectory);
    }

    public void printLastPose(){
        try{
            TrajectorySequence temp = trajList.get(trajList.size()-1);
            if(VERBOSE) {  RobotLog.dd(TAG, "Trajectory Sequence from %s to %s, dur=%f",
                    temp.start(), temp.end(), temp.duration());}
        }catch(Exception e){
            if(VERBOSE) { RobotLog.dd(TAG, "Unable to printLastPose");}
        }
    }

    public void newRoute(){

    }

    public void finalizeTrajSeq()
    {
        RobotLog.dd(TAG, "finalizing a TrajSeq");
        /* Finalize Traj Seq */
        if(null != trajList)
	    {
            RobotLog.dd(TAG,"trajList does not = null");
             if(null != traj)
	         {
                 trajList.add(traj.build());
             }
             if(null != fullSeq)
	         {
                 addTrajectory(fullSeq);
             }

             TrajectorySequenceBuilder full = new TrajectorySequenceBuilder(
                     firstPose,
                     defVelLim, defAccelLim,
                     RobotConstants.MAX_ANG_VEL, RobotConstants.MAX_ANG_ACCEL);

             double cumTime = 0.0;
            if(VERBOSE) { RobotLog.dd(TAG, "Done Building trajectories");}
             int seqNum = 0;
             TrajectorySequence parkTraj = parkMap.get(parkPos);
             if(null != parkTraj) {
                 trajList.add(parkTraj);
             }

             for (TrajectorySequence tSeq : trajList)
  	         {
                    String seqName = String.valueOf(seqNum++);
                    //TrajectorySequence tSeq = e.getValue();
                 if(VERBOSE) { RobotLog.dd(TAG, "Trajectory Sequence %s len=%d dur=%.2f",
                         seqName, tSeq.size(), tSeq.duration());}

                   for (int i = 0; i < tSeq.size(); i++)
  	          	   {
                         SequenceSegment seg = tSeq.get(i);
                       if(VERBOSE) {  RobotLog.dd(TAG, "Seg %d %s to %s in %.2f",
                           i, seg.getStartPose(), seg.getEndPose(), seg.getDuration());}

                         if (seg instanceof TrajectorySegment)
                         {
                               Trajectory t = ((TrajectorySegment) seg).getTrajectory();
                               full.addTrajectory(t);
                             if(VERBOSE) { RobotLog.dd(TAG, "TrajSeg %d %s to %s in %.2f",
                                 i, seg.getStartPose(), seg.getEndPose(), seg.getDuration());}

                               for (TrajectoryMarker m : ((TrajectorySegment) seg).getTrajectory().getMarkers())
                               {
                                   if(VERBOSE) { RobotLog.dd(TAG, "  marker: time=%.2f", m.getTime());}
                               }
                               cumTime += t.duration();
                         }
  	          		    else
  	          	        {
                             if (seg instanceof TurnSegment)
  	          		         {
                                   TurnSegment ts = (TurnSegment) seg;
                                   full.turn(ts.getTotalRotation());
                                 if(VERBOSE) { RobotLog.dd(TAG, "TurnSeg %d %s to %s in %.2f",
                                     i, seg.getStartPose(), seg.getEndPose(), seg.getDuration());}

                                   for (TrajectoryMarker m : seg.getMarkers())
  	          		               {
                                       full.addTemporalMarker(m.getTime(), m.getCallback());
                                   }
                                   cumTime += ts.getDuration();
                             }
  	          		         else if (seg instanceof WaitSegment)
  	          		         {
                                   WaitSegment ws = (WaitSegment) seg;
                                   full.waitSeconds(ws.getDuration());
                                 if(VERBOSE) {  RobotLog.dd(TAG, "WaitSeg %d %s to %s in %.2f",
                                     i, seg.getStartPose(), seg.getEndPose(), seg.getDuration());}

                                   for (TrajectoryMarker m : seg.getMarkers())
  	          		               {
                                       full.addTemporalMarker(m.getTime(), m.getCallback());
                                       if(VERBOSE) { RobotLog.dd(TAG, "  marker: time=%.2f", m.getTime());}
                                   }
                                   cumTime += ws.getDuration();
                              }
                         }
                    }
             }
         fullSeq = full.build();
         }
    }

    public Pose2d getStart(){
        return firstPose;
    }

    public enum SegmentState
    {
        MOVE_FROM_START,
		/* LEFT SIDE Trajectory Segments */
        DROP_CONE_3PT_JUNCTION,
        MOVE_FROM_3PT_JUNCTION,
        CONE_GRAB,
		/* RIGHT SIDE Trajectory Segments */
		MOVE_FROM_SHARED_5PT_JUNCTION,
		CONE_GRAB_FROM_SHARED_5PT_JUNCTION,
		/* END Of Specific Segments */		
        DROP_CONE_4PT_JUNCTION,
        REVERSE_FROM_4PT_JUNCTION,
        DROP_CONE_4PT_JUNCTION_REPEAT,
        REVERSE_FROM_4PT_JUNCTION_REPEAT,
        DROP_CONE_OPP_5PT_JUNCTION,
        PARK_RIGHT,
        PARK_LEFT,
        PARK_CENTER
    }
}