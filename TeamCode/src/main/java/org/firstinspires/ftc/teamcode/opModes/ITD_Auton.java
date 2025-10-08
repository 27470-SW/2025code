package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.field.ITD_Route;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.Route;
import org.firstinspires.ftc.teamcode.image.Detector;
import org.firstinspires.ftc.teamcode.image.ITD_Detector;
import org.firstinspires.ftc.teamcode.robot.MecanumBot;
import org.firstinspires.ftc.teamcode.robot.MecanumDriveLRR;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.BasicBot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.PreferenceMgr;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.EnumMap;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.field.Field.StartPos.*;

//@SuppressWarnings("ConstantConditions")
@SuppressWarnings("StatementWithEmptyBody")
@Config
@Autonomous(name="IntoTheDeepAuton", group="Auton")
//@Disabled
public class ITD_Auton extends InitLinearOpMode // implements FtcMenu.MenuButtons
{
    private static final String TAG = "SJH_PPA";



    ElapsedTime ltmr = new ElapsedTime();
    double ltimeout = 0.2;


    private static float delay;
    private static float xOffset;
    /* Allows pausing between each route state - useful for testing trajectory sequences 1 at a time */
    public static Field.AutonDebug autonDebug;

    public ITD_Auton()
    {
        RobotLog.dd(TAG, "PPAuto CTOR");
        alliance = Field.Alliance.valueOf(PreferenceMgr.getAllianceColor());
        startPos = Field.StartPos.values()[PreferenceMgr.getStartPosition()];
        parkPos = Field.Parks.values()[PreferenceMgr.getParkPosition()];
        delay    = PreferenceMgr.getDelay();
        xOffset  = PreferenceMgr.getXOffset();
        autonDebug  = Field.AutonDebug.values()[PreferenceMgr.getEnableAutonDebug()];
        firstLocation = Field.FirstLocation.values()[PreferenceMgr.getFirstLoc()];
        stackToBack = PreferenceMgr.getStackHighwayToBd();
        highways = new Field.Parks[]{PreferenceMgr.getHighway1(), PreferenceMgr.getHighway12(), PreferenceMgr.getHighway2(),PreferenceMgr.getHighway22(), PreferenceMgr.getHighway3(),PreferenceMgr.getHighway32()};
        pixelStacks = new Field.Parks[]{PreferenceMgr.getPixel1(), PreferenceMgr.getPixel2(), PreferenceMgr.getPixel3()};

        RobotConstants.init(chas, alliance, startPos, xOffset);
    }

    private void startMode()
    {
        dashboard.clearDisplay();
        do_main_loop();
    }

    //@SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() //throws InterruptedException
    {
        RobotLog.dd(TAG, "initCommon");
        useOpenCv = true;
        initCommon(this);

        if(CommonUtil.getInstance().hasCam)
        {
            RobotLog.dd(TAG, "We have a camera");
            camera = CommonUtil.camera;
        }

        setup();

        int initCycle = 0;
        int initSleep = 20;
        timer.reset();

        robot.setInInit(true);

        boolean inCamSetup = false;

        ElapsedTime advTmr = new ElapsedTime();
        double advTime = 1.5;

        det.setLogging(false);

        /* Before Play is pressed but after Init is triggered we remain here to perform any house keeping */
        while(!isStopRequested() && !isStarted())
        {
            /* Setting up bot from preference manager to run Auton */
            robot.update();

            gpad1.update();
            double camPos = RobotConstants.CAM_RED_1;
            if(startPos == START_SPECIMENS)
            {
                if(alliance == Field.Alliance.BLUE)
                    camPos = RobotConstants.CAM_BLU_1;
            }
            if(startPos == START_SAMPLES)
            {
                camPos = RobotConstants.CAM_RED_2;
                if(alliance == Field.Alliance.BLUE)
                    camPos = RobotConstants.CAM_BLU_2;
            }

            if(
                  (gpad1.just_pressed(ManagedGamepad.Button.A)) &&
                  (!gpad1.pressed(ManagedGamepad.Button.START))
              )
            {
                advTmr.reset();
                inCamSetup = true;

                if(camera != null)
                {
                    /* OpenCV starts the Webcam and starts capturing images
                    processFrame is called continously until the pipeline is paused
                    */
                    RobotLog.dd(TAG, "Setting image processing pipeline");
                    /* Camera is an OpenCV Camera that instantiated in commonUtil
                       To use to the camera we need to setPipeline to an OpenCV Pipeline -
                       Detector is a child OpenCV Pipeline
                     */
                    camera.setPipeline(det);
                    camera.resumeViewport();
                    det.setPaused(false);
                }
            }

            if(gpad1.just_pressed(ManagedGamepad.Button.B) &&
               !gpad1.pressed(ManagedGamepad.Button.START))
            {
                inCamSetup = false;
                if(camera != null)
                {
                    camera.pauseViewport();
                    camera.setPipeline(null);
                    det.setPaused(true);
                }
            }

            if(inCamSetup)
            {
                if(det instanceof ITD_Detector)
                {
                    ITD_Detector ffdet = (ITD_Detector)det;
                    initScanPos = ffdet.getPos();
                    initNumContours = ffdet.getNumContours();

                    if(advTmr.seconds() > advTime)
                    {
                        advTmr.reset();
                        //ffdet.toggleStage();
                        ffdet.advanceStage();
                        if (camera != null) {
                            camera.showFpsMeterOnViewport(false);
                        }

//                        boolean pipeLineRaw = ffdet.isPipeLineRaw();
//
//                        if(!pipeLineRaw)
//                        {
//                            camera.startStreaming(432, 240, OpenCvCameraRotation.UPRIGHT);
//                        }
//                        else
//                        {
//                            camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//                        }

                    }
                }
            }
            dashboard.displayText(2, "Scan: "
                                     + initScanPos + " " + initNumContours);

            if(initCycle % 10 == 0)
            {
                mechDrv.drawRoute();
//                double fhdg = robot.getHdg();
                StringBuilder motStr = new StringBuilder("ENCs:");
                for (Map.Entry<String, DcMotorEx> e : robot.motors.entrySet())
                {
                    if (e.getValue() == null) continue;
                    motStr.append(" ");
                    motStr.append(e.getKey());
                    motStr.append(":");
                    motStr.append(e.getValue().getCurrentPosition());
                }
                dashboard.displayText(6, motStr.toString());
//                dashboard.displayText(5, String.format(Locale.US, "FHDG %4.2f", fhdg));
                /*
                if (robot.colorSensor != null)
                {
                    int r = robot.getClrR();
                    int g = robot.getClrG();
                    int b = robot.getClrB();
                    float v = robot.getClrV();
                    RobotLog.dd(TAG, "RGB = %d %d %d V=%.2f", r, g, b, v);
                    dashboard.displayText(8,
                        String.format(Locale.US,"RGB %d %d %d V=%.2f", r, g, b, v));
                    String elevStr = "";
                    if(robot.elev != null) elevStr = robot.elev.toString();
                    dashboard.displayText(9, elevStr);
                }
                */
            }

            initCycle++;

            robot.finishFrame();

            robot.waitForTick(initSleep);
        }

        det.setLogging(true);

        det.reset();

        robot.setInInit(false);

        if(!isStopRequested())
        {
            startMode();
        }
        stopMode();
    }

    private void stopMode()
    {
        es.shutdownNow();

        if(camera != null)
        {
  //          camera.stopStreaming();
   //         camera.closeCameraDevice();
        }
    }

    private void setupBotComponents()
    {


    }

    /* @setup - Following is performed here
    *      - Builds Driver Station dashboard with values written in Auton Config
    *      - setups a the Auto Transitioner - this automatically launches the Teleop op mode upon exit of Power Play Auton
    *      - Initializes and builds all Trajectories
    *      - Initializes a Detector for image processing and enables cropping
    */
    private void setup()
    {
//        getPrefs();
        dashboard.displayText(0, "PLEASE WAIT - STARTING - CHECK DEFAULTS");
        dashboard.displayText(1, "Pref Values:");
        dashboard.displayText(2, "Pref BOT: " + robotName);
        dashboard.displayText(3, "Pref Alliance: " + alliance);
        dashboard.displayText(4, "Pref StartPos: " + startPos);
        dashboard.displayText(6,
            String.format(Locale.US, "Pref Delay: %.2f", delay));
        dashboard.displayText(7,
                              String.format(Locale.US, "Pref xOffset: %.2f", xOffset));
		dashboard.displayText(4, "Pref AutonDebug: " + autonDebug);
        logData = true;
        RobotLog.ii(TAG, "SETUP");

        dashboard.displayText(0, "INITIALIZING - Please wait");
        dashboard.displayText(6, "");
        dashboard.displayText(7, "");
        dashboard.displayText(14, String.format(Locale.US,"SW Ver SC Build 12_8_2022"));

        robot = new MecanumBot();

        //Since we only have 5 seconds between Auton and Teleop, automatically load
        //teleop opmode
        final String teleopName = "Mecanum";
        RobotLog.dd(TAG, "Setting up auto tele loader : %s", teleopName);
        //AutoTransitioner.transitionOnStop(this, teleopName);
        dashboard.displayText(1, "AutoTrans setup");

        BasicBot.curOpModeType = BasicBot.OpModeType.AUTO;

        robot.init(this, chas, true);
        try {
        if(robot.arm != null)
        {

                robot.initArmMot();

        }
        }catch (InterruptedException e) {
            RobotLog.dd(TAG, "Stopped mid init for arm encoder reset");
        }
        if(robot.claw != null) {
            robot.initClaw();
        }
        try {
            initAprilTag();
        }catch(Exception e)
        {
            RobotLog.ee(TAG, "Unable to initialize April Tag Camera");
        }

        RobotConstants.info();

        robot.setBcm(LynxModule.BulkCachingMode.MANUAL);
        mechDrv = (MecanumDriveLRR)(robot.drive);

        setupBotComponents();

        /* Build our Auton Trajectories */
        routeMain = new ITD_Route(robot, startPos, parkPos, firstLocation);

        resetBotLocation(routeMain.start);
        initHdg = routeMain.start.getHeading();
        BasicBot.DriveDir startDdir = BasicBot.DriveDir.PUSHER;
        robot.setDriveDir(startDdir);
        robot.setInitHdg(initHdg);
        robot.setAlliance(alliance);

        ePose = robot.drive.getPoseEstimate();
        robot.setAutonEndPos(new Point2d(ePose.getX(), ePose.getY()));
        robot.setAutonEndHdg(ePose.getHeading());

        dashboard.displayText(0, "GYRO CALIBRATING DO NOT TOUCH OR START");
        if (robot.imu != null)
        {
            gyroReady = robot.calibrateGyro();
        }
        dashboard.displayText(0, "GYRO CALIBATED: " + gyroReady);
        dashboard.displayText(1, "Robot Inited");

        det = new ITD_Detector(robotName);
        det.setCrop(true);

        setupLogger();
        dl.addField("Start: " + startPos.toString());
        dl.addField("Alliance: " + alliance.toString());
        RobotLog.ii(TAG, "STARTPOS %s", startPos);
        RobotLog.ii(TAG, "ALLIANCE %s", alliance);
        RobotLog.ii(TAG, "DELAY    %4.2f", delay);
        RobotLog.ii(TAG, "BOT      %s", robotName);
        RobotLog.dd(TAG, "Robot CPI " + RobotConstants.DT_CPI);
        RobotLog.dd(TAG, "BOTDIR=%s START_DDIR =%s",
            RobotConstants.DT_DIR, startDdir);

        dashboard.displayText(0, "READY TO START");
    }


    private void performTrajSeq(TrajectorySequence seq, String seqName)
    {
        if(autonDebug == Field.AutonDebug.ENABLE)
        {
            /* log to the Driver Station dashboard during Auton Debug */
            int lnNum = 5;

            dashboard.displayText(lnNum, String.format(Locale.US, "Traj Seq %s len=%d dur=%.2f",
                    seqName, seq.size(), seq.duration()));

            for (int i = 0; i < seq.size(); i++)
            {
                SequenceSegment seg = seq.get(i);

                dashboard.displayText(lnNum++, String.format(Locale.US, "Seg %d %s to %s in %.2f",
                        i, seg.getStartPose(), seg.getEndPose(), seg.getDuration()));


                if (seg instanceof TrajectorySegment)
                {
                    dashboard.displayText(lnNum++, String.format(Locale.US, "TrajSeg %d %s to %s in %.2f",
                            i, seg.getStartPose(), seg.getEndPose(), seg.getDuration()));

                    for (TrajectoryMarker m : ((TrajectorySegment) seg).getTrajectory().getMarkers()) {
                        dashboard.displayText(lnNum++, String.format(Locale.US, "  marker: time=%.2f", m.getTime()));
                    }
                }
                else
                {
                    if (seg instanceof TurnSegment)
                    {
                        dashboard.displayText(lnNum++, String.format(Locale.US, "TurnSeg %d %s to %s in %.2f",
                                i, seg.getStartPose(), seg.getEndPose(), seg.getDuration()));
                    }
                    else if (seg instanceof WaitSegment)
                    {
                        dashboard.displayText(lnNum++, String.format(Locale.US, "WaitSeg %d %s to %s in %.2f",
                                i, seg.getStartPose(), seg.getEndPose(), seg.getDuration()));
                    }
                    for (TrajectoryMarker m : seg.getMarkers())
                    {
                        dashboard.displayText(lnNum++, String.format(Locale.US, "  marker: time=%.2f", m.getTime()));
                    }
                }
            }
        }

        /* Waits here until game pad button A is pressed before proceeding to the next sequence
         *  Useful for debugging, especially when the bot is not where you would expect it to be.
         *  */
        while(
                (autonDebug == Field.AutonDebug.ENABLE) &&
                (opModeIsActive())
             )
        {
            gpad1.update();
            if(gpad1.just_pressed(ManagedGamepad.Button.A))
            {
                break;
            }
            robot.finishFrame();
            robot.waitForTick(20);
        }

        RobotLog.ii(TAG, "Driving trajectorySeq %s at %.2f",
                seqName, startTimer.seconds());

        /* Here is where the drivetrain/route magic happens */
        if (opModeIsActive())
        {
            mechDrv.followTrajectorySequenceAsync(seq);
        }

        while (opModeIsActive() && !isStopRequested() && mechDrv.isBusy())
        {
            robot.update();

            /* Get the current pose each frame and save it to a static variable
             * to allow teleop to know where it is when it starts
             */
            ePose = robot.drive.getPoseEstimate();
            robot.setAutonEndPos(new Point2d(ePose.getX(), ePose.getY()));
            robot.setAutonEndHdg(ePose.getHeading());
/*
                /* Check sensors before deciding next sequence */
/*                if(robot.intake != null)
                {
                    boolean actTrig = robot.intake.getActionTriggered();
                    if (s == Route.State.PREGRAB0 || s == Route.State.PREGRAB1)
                    {
                        hasFreight |= actTrig;
                        if(actTrig && !lastActTrig)
                            RobotLog.dd(TAG, "Detected freight action trigger in pregrab");
                    }
                    if (s == Route.State.GRAB0 || s == Route.State.GRAB1)
                    {
                        hasFreight |= actTrig;
                        if(actTrig && !lastActTrig)
                            RobotLog.dd(TAG, "Detected freight action trigger in grab");
                    }
                    lastActTrig = actTrig;
                }
*/
/*
                if(robot.colorSensor !=null)
                {
                    boolean clrFnd = robot.getClrV() > RobotConstants.CL_VALUE_THRESH;
                    if(!lastColorLine && clrFnd)
                    {
                        RobotLog.dd(TAG, "Line detected? clrV:%.2f at %s",
                                    robot.getClrV(), ePose.toString());
                    }
                    lastColorLine = clrFnd;
                }

 */
            robot.finishFrame();
        }
    }

    private void do_main_loop()
    {
        startTimer.reset();
        dl.resetTime();
        if (camera != null) {
            camera.setPipeline(det);
        }

        RobotLog.ii(TAG, "STARTING AT %.2f %.2f", startTimer.seconds(), timer.seconds());
        if (logData)
        {
            Pose2d spt = routeMain.start;
            dl.addField("START");
            dl.addField(initHdg);
            dl.addField(spt.getX());
            dl.addField(spt.getY());
            dl.newLine();
        }
        /* Send Telemetry Data to the Dashboard */
        logFtcDashboard();

        RobotLog.ii(TAG, "Delaying for %4.2f seconds", delay);
        ElapsedTime delayTimer = new ElapsedTime();
        while (opModeIsActive() && delayTimer.seconds() < delay)
        {
            idle();
        }

        RobotLog.ii(TAG, "Done delay");

//        RobotLog.ii(TAG, "START CHDG %6.3f", robot.getGyroHdgDeg());

        RobotLog.ii(TAG, "Action SCAN_IMAGE");
        route = routeMain;
/*
        doScan();

        if (detectedTeamElementPosition == ITD_Detector.Position.LEFT)
        {
            route = routeLeft;

        }
        else if (detectedTeamElementPosition == ITD_Detector.Position.RIGHT)
        {
            route = routeRight;

        }
        else
        {
            route = routeCenter;

        }*/
        try {
            int lnum = 8;
            dashboard.displayText(lnum++, "Start:    " + startPos);
            dashboard.displayText(lnum++, "Park Position:  " + parkPos);
            dashboard.displayText(lnum++, "First Location:  " + firstLocation);
//                dashboard.displayText(lnum++, "Team Element:    " + route.teamElement);
            dashboard.displayText(lnum++, "Curcuit 1 " + highways[0] + "," + pixelStacks[0] + "," + highways[1]);
            dashboard.displayText(lnum++, "Curcuit 2 " + highways[2] + "," + pixelStacks[1] + "," + highways[3]);
            dashboard.displayText(lnum++, "Curcuit 3 " + highways[4] + "," + pixelStacks[2] + "," + highways[5]);
        }catch (Exception e){
            try{
                dashboard.displayText(12, "Failure to print information");
            }catch(Exception e2){}
        }

        if (route == null)
        {
            RobotLog.ee(TAG, "Route Was Not Initialized!");
        }

        timer.reset();


        /*if(startPos == START_LEFT)
        {
            if(detectedParkPosition == PPlayDetector.Position.RIGHT)
                ltimeout = RobotConstants.DP_PREWAIT_H;
            else if(detectedParkPosition == PPlayDetector.Position.CENTER)
                ltimeout = RobotConstants.DP_PREWAIT_M;
        }*/

        boolean hasFreight = false;
        boolean lastActTrig = false;

        /* There is a state for each "chunk" of our overall auton route.
        *  Each state gets assigned a trajectorySequence.
        *  Here, we loop thru each trajectorySequence - starting it asynchronously
        *  with mechDrv.followTrajectorySequenceAsync(seq) below.
        *  Then we loop until the sequence is complete - updating the robot and
        *  saving the current pose each time through the loop.
        *  Note that there also some delay/loops before certain states/sequences
        *  to allow us to safely and asynchronously wait for some condition to get
        *  satisfied (i.e. elevator up, freight intake detected, etc)
        */

        doAutonFromInitTrajectories2();
        doAutonFromInitTrajectories();
        doAutonParking();


        RobotLog.dd(TAG, "Finished auton segments at %s", ePose.toString());

        ePose = robot.drive.getPoseEstimate();
        robot.setAutonEndPos(new Point2d(ePose.getX(), ePose.getY()));
        robot.setAutonEndHdg(ePose.getHeading());
        RobotLog.dd(TAG, "Exiting auton at %s", ePose.toString());
    }

    private void doAutonFromInitTrajectories()
    {
        boolean skipNext = false;
        /*
		   Trajectories for State Map for Lake Orion Competition 
		   Below for loop needs to be modified to use the Lake Orion Competition
		   EnumMap<org.firstinspires.ftc.teamcode.field.Route.State, TrajectorySequence> stateMap = route.stateTrajMap;
		*/   
		EnumMap<Route.SegmentState, TrajectorySequence> stateMap = route.segmentStateMap;

		
		
        for (Map.Entry<Route.SegmentState, TrajectorySequence> e : stateMap.entrySet())
        {
            org.firstinspires.ftc.teamcode.field.Route.SegmentState s = e.getKey();
            String seqName = s.name();
            TrajectorySequence seq = e.getValue();

            dashboard.displayText(4, String.format(Locale.US,"Seq %s",
                    seqName));

            RobotLog.dd(TAG, "Transitioning to state %s", seqName);

            if(skipNext)
            {
                skipNext = false;
            }
 /*         To be removed. Left here as reference to as an example of how to insert synchronous tasks
            if(s == Route.State.PREGRAB0 || s == Route.State.PREGRAB1)
            {
                hasFreight = false;
            }
*/
            /* Parking is the last route that shall be processed in the route map.
            *  All Parking routes trajectories will be appended to the route state map
            *  However, only 1 parking maneuver will be executed.
            *  Parking Position is determined by scanned Image on the signal sleeve.
            */
            if(
                    (s == Route.SegmentState.PARK_RIGHT) ||
                    (s == Route.SegmentState.PARK_CENTER) ||
                    (s == Route.SegmentState.PARK_LEFT)
             )
            {
                if(
                        (s == Route.SegmentState.PARK_RIGHT && detectedTeamElementPosition != ITD_Detector.Position.RIGHT) ||
                        (s == Route.SegmentState.PARK_CENTER && detectedTeamElementPosition != ITD_Detector.Position.CENTER) ||
                        (s == Route.SegmentState.PARK_LEFT && detectedTeamElementPosition != ITD_Detector.Position.LEFT)
                  )
                {
                    /* Skip the route state because the position detected does not match */
                    RobotLog.dd(TAG, "Skip Parking %s", s.name());
                    continue;
                }
                else
                {
                    RobotLog.dd(TAG, "Parking at %s", detectedTeamElementPosition);
                }

            }

            /* This block is to do safe/async delay to allow arm to get to level */

/*          An example of inserting a wait to ensure some has been performed.
            Example: move the elevator/lift to level for certain amount of time.
            Hopefully, we have a really fast elevator and this won't be required.
*/

//            if(0)
//                    //(s == org.firstinspires.ftc.teamcode.field.Route.State.BACK_AWAY_4PTJUNCT_1) ||
//                    //(s == org.firstinspires.ftc.teamcode.field.Route.State.BACK_AWAY_4PTJUNCT_2) ||
//                    //(s == org.firstinspires.ftc.teamcode.field.Route.State.AWAY_FROM_CONES_1)
//
//            {
//                ltmr.reset();
//                RobotLog.dd(TAG, "Waiting for lift to %s on %s for %.2f",
//                        detectedParkPosition, seqName, ltimeout);
//                while (
//                        (opModeIsActive()) &&
//                        (!isStopRequested()) &&
//                        (ltmr.seconds() < ltimeout)
//                )
//                {
//                    robot.update();
//                    /* if(robot.elev != null && !robot.elev.getMovingToLevel())
//                    {
//                        RobotLog.dd(TAG, "Elev not moving to level");
//                        break;
//                    }*/
//                    idle();
//                    robot.finishFrame();
//                    //robot.waitForTick(20);
//                }
//            }

            /* This block replaces a fixed waitSeconds segment in trajectory sequences
            *  Decision logic can be inserted to do tasks between the trajectories,
            *  like waiting for the elevator to rise to a certain level or figuring out where to park.
            *  The trajectory that just finished will have turned on the intake
            */
/*            if(s == Route.State.PREDROP1 || s == Route.State.PREDROP2)
            {
                if(timer.seconds() > 24.0)
                {
                    RobotLog.dd(TAG, "Time running out - skippin remaining segments");
                    break;
                }

                RobotLog.dd(TAG, "Waiting for freight or timeout");
                ltmr.reset();
                while(opModeIsActive() && !isStopRequested() &&
                      ltmr.seconds() < RobotConstants.IN_TIMEOUT)
                {
                    robot.update();
                    if(robot.intake != null && robot.intake.getActionTriggered())
                    {
                        RobotLog.dd(TAG, "Detected freight action trigger in wait");
                        hasFreight = true;
                        break;
                    }
                    idle();
                    robot.finishFrame();
                    robot.waitForTick(20);
                }
            }
*/
            performTrajSeq(seq, seqName);

        }

    }

    private void resetBotLocation(Pose2d start){
        robot.drive.setPoseEstimate(start);

    }

    private void doAutonFromInitTrajectories2()
    {
        RobotLog.dd(TAG, "In doAutonFromInitTrajectories2");
        route.runRoute(this);
    }


    public void doAuton(Route autonRoute)
    {
        RobotLog.dd(TAG, "In doAuton");
        int trajNum = 0;

        if(null != autonRoute.trajList)
        {
            resetBotLocation(autonRoute.trajList.get(0).start());
            for (TrajectorySequence tSeq : autonRoute.trajList)
            {
                trajNum++;

                String seqName = String.format(Locale.US, "Seq %d",
                        trajNum);
                dashboard.displayText(4, seqName);

                performTrajSeq(tSeq, seqName);
            }
        }
        else
        {
            RobotLog.dd(TAG, "trajList is null");
        }
    }

    private void doAutonParking(){
        /*
        TrajectorySequence parkSeq = route.parkMap.get(detectedTeamElementPosition);
        if(null != parkSeq) {
            String seqName = "unknown";
            if (ITD_Detector.Position.CENTER == detectedTeamElementPosition) {
                seqName = "CENTER_PARK";
            } else if (ITD_Detector.Position.LEFT == detectedTeamElementPosition) {
                seqName = "LEFT_PARK";
            } else //(PPlayDetector.Position.RIGHT == detectedParkPosition)
            {
                seqName = "RIGHT_PARK";
            }


            //Temporary
            RobotLog.dd(TAG, "detectedParkPosition, %s", detectedTeamElementPosition.name());
            for (int i = 0; i < parkSeq.size(); i++) {
                SequenceSegment seg = parkSeq.get(i);
                RobotLog.dd(TAG, "Seg %d %s to %s in %.2f",
                        i, seg.getStartPose(), seg.getEndPose(), seg.getDuration());

                if (seg instanceof TrajectorySegment) {
                    Trajectory t = ((TrajectorySegment) seg).getTrajectory();
                    RobotLog.dd(TAG, "TrajSeg %d %s to %s in %.2f",
                            i, seg.getStartPose(), seg.getEndPose(), seg.getDuration());
                    for (TrajectoryMarker m : ((TrajectorySegment) seg).getTrajectory().getMarkers()) {
                        RobotLog.dd(TAG, "  marker: time=%.2f", m.getTime());
                    }
                } else {
                    if (seg instanceof TurnSegment) {
                        TurnSegment ts = (TurnSegment) seg;
                        RobotLog.dd(TAG, "TurnSeg %d %s to %s in %.2f",
                                i, seg.getStartPose(), seg.getEndPose(), seg.getDuration());
                    } else if (seg instanceof WaitSegment) {
                        WaitSegment ws = (WaitSegment) seg;
                        RobotLog.dd(TAG, "WaitSeg %d %s to %s in %.2f",
                                i, seg.getStartPose(), seg.getEndPose(), seg.getDuration());
                    }
                }
            }

            dashboard.displayText(4, seqName);
            performTrajSeq(parkSeq, seqName);
        }
*/
    }

    private void doScan()
    {
        RobotLog.dd(TAG, "doScan");
        if (camera != null){
        detectedTeamElementPosition =  getImageTargetPos();
        }
        RobotLog.dd(TAG, "doScan scanPos = %s", detectedTeamElementPosition);

        setScanPoint();

    }

    private void setScanPoint()
    {
        RobotLog.dd(TAG, "Getting scanPoint for %s %s %s",
                alliance, startPos, detectedTeamElementPosition);
    }

    @SuppressWarnings("unused")
    private double angNormalize(double ang)
    {
        double ret = ang;
        while (ret >   180) ret -= 360;
        while (ret <= -180) ret += 360;
        return ret;
    }

    private ITD_Detector.Position getImageTargetPos()
    {
        if(!opModeIsActive()) return detectedTeamElementPosition;

        if(camera == null)
        {
            return ITD_Detector.Position.CENTER;
        }
        ITD_Detector.Position tmpScanPos = ITD_Detector.Position.NONE;

        double scanTimeout = 0.5;

        ElapsedTime mtimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(opModeIsActive()                    &&
              tmpScanPos == ITD_Detector.Position.NONE &&
              mtimer.seconds() < scanTimeout)
        {
            RobotLog.dd(TAG, "getScanPos - loop calling det.logDebug");
            if(det instanceof ITD_Detector)
                tmpScanPos = ((ITD_Detector) det).getPos();

            if(tmpScanPos == ITD_Detector.Position.NONE)
                sleep(10);
        }
            if (camera != null){
                camera.setPipeline(null);
                camera.stopStreaming();
                det.saveImages();
            }

        dashboard.displayText(2, "scanPos: " + tmpScanPos);
        RobotLog.dd(TAG, "scanPos = %s", tmpScanPos);
        if (tmpScanPos == ITD_Detector.Position.NONE) {
            if (alliance == Field.Alliance.RED && startPos == START_SPECIMENS || alliance == Field.Alliance.BLUE && startPos == START_SPECIMENS) {
                    RobotLog.dd(TAG, "No image answer found - defaulting to RIGHT");
                    tmpScanPos = ITD_Detector.Position.RIGHT;
                    dashboard.displayText(6, "No image answer found " + tmpScanPos);
            } else if (alliance == Field.Alliance.BLUE && startPos == START_SAMPLES || alliance == Field.Alliance.RED && startPos == START_SPECIMENS) {
                    RobotLog.dd(TAG, "No image answer found - defaulting to LEFT");
                    tmpScanPos = ITD_Detector.Position.LEFT;
                    dashboard.displayText(6, "No image answer found " + tmpScanPos);
            } else {
                    RobotLog.dd(TAG, "No image answer found - defaulting to CENTER");
                    tmpScanPos = ITD_Detector.Position.CENTER;
                    dashboard.displayText(6, "No image answer found " + tmpScanPos);
            }
        }

        return tmpScanPos;
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
//        // Create the AprilTag processor by using a builder.
//        aprilTag = new AprilTagProcessor.Builder().build();
//
//        // Adjust Image Decimation to trade-off detection-range for detection-rate.
//        // eg: Some typical detection data using a Logitech C920 WebCam
//        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
//        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
//        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
//        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
//        // Note: Decimation can be changed on-the-fly to adapt during a match.
//        aprilTag.setDecimation(2);
//
//        // Create the vision portal by using a builder.
//        if (USE_WEBCAM) {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(hardwareMap.get(WebcamName.class, "webcamREAR"))
//                    .addProcessor(aprilTag)
//                    .build();
//            setManualExposure(5, 250);  // Use low exposure time to reduce motion blur
//        } else {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(BuiltinCameraDirection.BACK)
//                    .addProcessor(aprilTag)
//                    .build();
//        }
//    }
//
//    /*
//     Manually set the camera gain and exposure.
//     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
//    */
//    private void    setManualExposure(int exposureMS, int gain) {
//        // Wait for the camera to be open, then use the controls
//
//        if (visionPortal == null) {
//            return;
//        }
//
//        // Make sure camera is streaming before we try to set the exposure controls
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
//            telemetry.update();
//            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
//        }
//
//        // Set camera controls unless we are stopping.
//        if (!isStopRequested())
//        {
//            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                sleep(50);
//            }
//            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
//            sleep(20);
//            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//            gainControl.setGain(gain);
//            RobotLog.dd(TAG, String.format("Min Gain: %d, Max Gain: %d", gainControl.getMinGain(), gainControl.getMaxGain()));
//            sleep(20);
//        }
    }


    private void setupLogger()
    {
        if (logData)
        {
            dl.addField("NOTE");
            dl.addField("FRAME");
            dl.addField("Gyro");
            dl.addField("LENC");
            dl.addField("RENC");
            dl.addField("LPWR");
            dl.addField("RPWR");
            dl.addField("RED");
            dl.addField("GRN");
            dl.addField("BLU");
            dl.addField("ESTX");
            dl.addField("ESTY");
            dl.addField("ESTH");
            dl.newLine();
        }
    }



    private void logFtcDashboard()
    {
        TelemetryPacket packet = cmu.getTelemetryPacket();
        if(packet == null)
        {
            packet = new TelemetryPacket();
            cmu.setTelemetryPacket(packet);
        }
        FtcDashboard dbd = CommonUtil.getInstance().getFtcDashboard();
        packet.put("pos", robot.logIntakeCurSpd);
        packet.put("Spd", robot.logIntakeCurSpd);

        dbd.sendTelemetryPacket(packet);
    }

    private MecanumBot robot;
    private MecanumDriveLRR mechDrv;
    private Pose2d ePose = new Pose2d();

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime startTimer = new ElapsedTime();

    private Detector det;
    private ITD_Detector.Position detectedTeamElementPosition = ITD_Detector.Position.NONE;
    private ITD_Detector.Position initScanPos = ITD_Detector.Position.NONE;
    private int initNumContours = 0;

    private double initHdg = 0.0;
    private boolean gyroReady;

    @SuppressWarnings("unused")
    private final boolean useImageLoc  = false;

    private static final boolean useLight = true;
    private static final boolean usePhone = false;
    private OpenCvCamera camera;

    private boolean lastColorLine = false;

    private final ExecutorService es = Executors.newSingleThreadExecutor();

    private ITD_Route route;
    private ITD_Route routeMain;
    private ITD_Route routeLeft;
    private ITD_Route routeCenter;



}