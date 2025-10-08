package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.ITD_Route;
import org.firstinspires.ftc.teamcode.field.SpinRoute;
import org.firstinspires.ftc.teamcode.field.SpinRoutebasket;
import org.firstinspires.ftc.teamcode.robot.MecanumBot;
import org.firstinspires.ftc.teamcode.robot.MecanumDriveLRR;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.BasicBot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Input_Shaper;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;
import java.util.Timer;
import java.util.concurrent.TimeUnit;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.ARM_MAX_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.ARM_MIN_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.ARM_SPD;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_LEVS;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_SPD;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.POSE_EQUAL;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.SLIDECPI;


@Config
@TeleOp(name = "Mecanum")
//@Disabled
public class MecanumTeleop extends InitLinearOpMode
{
    private static final String TAG = "SJH_MTD";

    public void initPreStart() throws InterruptedException
    {
        // Initialize the Apriltag Detection process
        try {
            initAprilTag();
        }catch(Exception e)
        {
            RobotLog.ee(TAG, "Unable to initialize April Tag Camera");
        }
        BasicBot.OpModeType prevOpModeType = BasicBot.curOpModeType;
        BasicBot.curOpModeType = BasicBot.OpModeType.TELE;

        /* Initialize the hardware variables. */
        if(VERBOSE) {  RobotLog.dd(TAG, "Initialize robot");}


        if(VERBOSE) {  RobotLog.dd(TAG, "Prev opmode type=%s.", prevOpModeType);}
        /* Always initialize the sensors like the IMU and color sensors,
         *  even if you are Auto transitioning from another OpMode
        */
        robot.init(this, chas, true);
        robot.setBcm(LynxModule.BulkCachingMode.MANUAL);

/*        armButton = hardwareMap.get(TouchSensor .class, "armL0");

 */
//        if(robot.claw != null)
//        {
//            /* keep the claw Open in case we have a cone in our grasp */
//            robot.claw.openClaw();
//        }
        Pose2d startPose = new Pose2d();
        if(prevOpModeType != BasicBot.OpModeType.AUTO)
        {
            try {
                if (robot.slides != null) {
                    robot.initSlides();
                }
                if (robot.arm != null) {
                    robot.initArmMot();
                }
            } catch (Exception e) {

            }
            // fo testin
//            Thread.sleep(3000);
//            robot.arm.moveToLevel(3,.5);
//            robot.slides.moveToLevel(3);
//            robot.armLevel = 3;
//            robot.slideLevel = 3;

        }
        else
        {
            Point2d autonEndPos = robot.getAutonEndPos();
            double autonEndHdg = robot.getAutonEndHdg();
            startPose = new Pose2d(autonEndPos.getX(), autonEndPos.getY(), autonEndHdg);
            if(VERBOSE) {  RobotLog.dd(TAG, "Start Aend fHdg %.2f", Math.toDegrees(autonEndHdg));
            RobotLog.dd(TAG, "Start Pos %s", autonEndPos.toString());}
        }








        /* Opportunity for a telop assist using Trajectory sequences */

        //route = new PPlayRoute(robot, START_RIGHT, robot.getAlliance());
/*      I thought that this was a timing issue, but putting in a 5 sec wait did not help
        
        ElapsedTime imuWait = new ElapsedTime();
        double imuInitWaitTime = 5.0;

        imuWait.reset();
        RobotLog.dd(TAG, "Waiting for IMU BNO055 Device to iniatilize");
                ;
        while (imuWait.seconds() < imuInitWaitTime)
        {
            idle();
            robot.waitForTick(20);
        }
*/

        mechDrv = (MecanumDriveLRR)(robot.drive);
        mechDrv.setPoseEstimate(startPose);
        mechDrv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        for(String k : robot.motors.keySet())
        {
            if (robot.motors.get(k) == null) continue;
            if(VERBOSE) {  RobotLog.dd(TAG, "Motor mode in teleop = " + k + ":" +
                robot.motors.get(k).getMode());}
        }

        lBumperPressed = false;

        clawLev = 0;
        armNslidesLev = 0;
        robot.claw.Triggr = true;

    }

    public int clawLev;
    public int armNslidesLev;
    public boolean pixelDetected = false;
    private int ONE_CONSTANT = 1;

    void processSensors(){

    }

    private void update()
    {
        robot.update();

        l = 0;

        cnts=robot.getCnts();
        vels=robot.getVels();
//        hdg=robot.getHdg();

        poseEstimate = robot.drive.getPoseEstimate();
        processSensors();

    }

    protected static boolean VERBOSE = RobotConstants.logVerbose;
    private void printTelem()
    {

        String cntStr = String.format(Locale.US,"CNTS: %d %d %d %d",
                cnts[0], cnts[1], cnts[2], cnts[3]);
        String velStr = String.format(Locale.US,"VELS: %d %d %d %d",
                (int)vels[0], (int)vels[1], (int)vels[2], (int)vels[3]);
        String posStr = String.format(Locale.US, "X:%.2f Y:%.2f H:%.2f",
            poseEstimate.getX(), poseEstimate.getY(), Math.toDegrees(poseEstimate.getHeading()));

     //   dashboard.displayText(l++, "Dir " + robot.getDriveDir());
        dashboard.displayText(l++, posStr);
        dashboard.displayText(l++, cntStr);
        dashboard.displayText(l++, velStr);
       // dashboard.displayText(l++, lStr);
        //dashboard.displayText(l++, aStr);
        dashboard.displayText(l++, String.format(Locale.US,"L_IN %4.2f L %4.2f", raw_lr, lr));
        dashboard.displayText(l++, String.format(Locale.US,"R_IN %4.2f R %4.2f", raw_fb, fb));
        dashboard.displayText(l++, String.format(Locale.US,"T_IN %4.2f T %4.2f", raw_turn, turn));

        dashboard.displayText(l++, String.format(Locale.US,"Mrk Pos %.2f MvRate %.2f ",
                lastMrkPos, moveAtRate));
        if(null != robot) {
            if (null != robot.slides) {
                dashboard.displayText(l++, String.format(Locale.US, "elevator encoding %d", robot.slides.getLiftPos()));
            }
            if (null!= robot.rearDistSensor){

                dashboard.displayText(l++, String.format(Locale.US, "rear distance %f", robot.rearDistSensor.getDistance(DistanceUnit.CM)));
            }
        }
        dashboard.displayText(l++, String.format(Locale.US, "lyftpowr %4.2f", liftSpd ));
        dashboard.displayText(14, String.format(Locale.US,"SW Ver SC Build 12_8_2022"));
        //dashboard.displayText(l++,String.format(Locale.US, "PixelDistance: %f", robot.colorFindDistance()));
        dashboard.displayText(l++, String.format(Locale.US, "arm encoder: %d", robot.arm.getCurEnc() ));
        dashboard.displayText(l++, String.format(Locale.US, "arm limit switch value: %b", armButton.isPressed()));

        if(VERBOSE) RobotLog.dd(TAG, "TEL SHT:%.1f ARM:%.1f INT:%.1f DRV:%.1f",
            spinTime, liftTime, intTime, drvTime);
        if(VERBOSE) RobotLog.dd(TAG, "TEL U:%.1f C:%.1f D:%.1f P:%.1f L:%.1f F:%.1f W:%.1f",
            u, c, d, p, L, f, w);


    }

    private int logLoop = 0;
    public int LOGMILLISCYCLES = 100;
    private void doLogging(double millis)
    {
        if(VERBOSE){
            TelemetryPacket packet = cmu.getTelemetryPacket();
            if(packet == null)
            {
                packet = new TelemetryPacket();
                cmu.setTelemetryPacket(packet);
            }

            //Can put stuff before draw call like:
            // packet.put("pos", pos);
            mechDrv.draw();
        }else{
            if(0 != millis) {
                logLoop++;
                if (logLoop >= LOGMILLISCYCLES) {
                    logLoop = 0;
                    RobotLog.dd(TAG, String.format("Cycle Time: %f", millis));
                }
            }
        }

    }

    double liftSpd = 0.0;
    int stackLvlCnt = 0;
    boolean joystickUsed = false;
    int targetEncoder = 0;

    private void controlArm()
    {


        if(robot.arm == null) return;
        boolean start = gpad2.pressed(ManagedGamepad.Button.START);
        double lftPwr = -gpad2.value(ManagedGamepad.AnalogInput.R_STICK_Y);
        /* Move the Elevator to desired HuB level */
       // boolean armLevelUp   = gpad2.just_pressed(ManagedGamepad.Button.D_UP);
        //boolean armLevelDown   = gpad2.just_pressed(ManagedGamepad.Button.D_DOWN);
        if (lftPwr >= -.1 && lftPwr <= .1)
        {
            if(robot.arm.getMode() == RUN_TO_POSITION)
            {


            }
            else
            {
                if(joystickUsed == true){
                    targetEncoder = robot.arm.getCurEnc();
                    joystickUsed = false;
                }
                robot.arm.moveToCnt(targetEncoder, ARM_SPD);
                RobotLog.dd(TAG, "setting arm to encoder: %d", targetEncoder);

                if(VERBOSE) { dashboard.displayText(13, String.format(Locale.US, "Target Encoder %d",targetEncoder));}
            }

        }
        else
        {
            robot.arm.setMode(RUN_USING_ENCODER);
            liftSpd = lftPwr;
            joystickUsed = true;
            RobotLog.dd(TAG, "setting arm mode to RUN_USING_ENCODER");

            //robot.elev.moveToCnt(robot.elev.getCurEnc(), RobotConstants.EL_SPD);
        }

        int offset = 1;
        //not working. why you ask? figure it out.
//        if (robot.elbowMotor.getCurEnc() < -1600 && robot.elbowMotor.getCurEnc() > -1675){
//            stackLvlCnt = 2;
//        }
//        if (robot.elbowMotor.getCurEnc() < -1800 && robot.elbowMotor.getCurEnc() > -1900){
//            stackLvlCnt = 3;
//        }
//        if (robot.elbowMotor.getCurEnc() < -2100 && robot.elbowMotor.getCurEnc() > -2200){
//            stackLvlCnt = 4;
//        }
/*        if (armLevelUp)
        {
            if (stackLvlCnt < EL_NUM_LEVS -1)
            {
                stackLvlCnt++;
                robot.arm.moveToLevel(stackLvlCnt, EL_SPD);

            }
            else
            {

            }

            if(VERBOSE) {  dashboard.displayText(13, String.format(Locale.US, "D up Pressed %d",stackLvlCnt));}
        }
        else if (armLevelDown)
        {
            if (stackLvlCnt > 0)
            {
                stackLvlCnt--;
                robot.arm.moveToLevel(stackLvlCnt, EL_SPD_DWN);

            }
            else
            {
                stackLvlCnt = EL_NUM_LEVS - 1;
                robot.arm.moveToLevel(stackLvlCnt, EL_SPD);
            }

            if(VERBOSE) {  dashboard.displayText(13, String.format(Locale.US, "D down Pressed %d",stackLvlCnt));}
        }
 */
        try{
            if (robot.arm.getMode() != RUN_TO_POSITION) {
                double armLimit = Math.min(ARM_MAX_ENCODER, getArmSoftLimit());
                RobotLog.dd(TAG, "encoder = %d, MIN = %d, MAX = %d, the arm's mode: %s", robot.arm.getCurEnc(), ARM_MIN_ENCODER, ARM_MAX_ENCODER, robot.arm.getMode().name());
                if (
                        (liftSpd <= 0 && robot.arm.getCurEnc() > ARM_MIN_ENCODER) ||
                                (liftSpd >= 0 && robot.arm.getCurEnc() < armLimit)
                ) {
                    double locSpeedLimit = 1;
//                if(liftSpd <= -.1) {
//                    locSpeedLimit = .5;
//                    if (robot.elbowMotor.getCurEnc() <= 300) {
//                        locSpeedLimit = .3;
//                    } else if (robot.elbowMotor.getCurEnc() <= 200) {
//                        locSpeedLimit = .15;
//                    } else if (robot.elbowMotor.getCurEnc() <= 100) {
//                        locSpeedLimit = .075;
//                    }
//                }

                    robot.arm.moveAtControlRate(EL_SPD * liftSpd * locSpeedLimit);
                    RobotLog.dd(TAG, "EL_SPD= %f, lftspd = %f, locSpeed = %f", EL_SPD, liftSpd, locSpeedLimit);
                } else {
                    robot.arm.moveAtControlRate(0);
                }

            }
        }
        catch(Exception e)
        {}
/*
        robot.setClawPos(gpad2.value(ManagedGamepad.AnalogInput.R_STICK_Y));
            if(gpad2.pressed(ManagedGamepad.Button.D_RIGHT))
            {
                robot.setClawPos(.2);
                if(VERBOSE) { dashboard.displayText(12, String.format(Locale.US, "manually adjusting wrist"));}
            }
            if(gpad2.pressed(ManagedGamepad.Button.D_LEFT))
            {
                robot.setClawPos(-.2);
                if(VERBOSE) { dashboard.displayText(12, String.format(Locale.US, "manually adjusting wrist"));}
            }
*/
        if(VERBOSE) { dashboard.displayText(8, String.format(Locale.US, "extender motor: %d", robot.slides.getLiftPos()));
        dashboard.displayText(15, String.format(Locale.US, "arm level: %d, encoder: %d, power: %f ",robot.armLevel,robot.arm.getCurEnc(),robot.arm.getPwr()));}
    }


 
    private void controlIntake()
    {
        double intakePwr = -gpad2.value(ManagedGamepad.AnalogInput.L_STICK_Y);
        //RobotLog.dd(TAG, "intake");
        if (intakePwr >= -.1 && intakePwr <= .1)
        {
            //RobotLog.dd(TAG, "intake not used");

        robot.intake.setPwr(0);
        }else
        {
            RobotLog.dd(TAG, "working intake");

            robot.intake.setPwr(intakePwr);
        }
      
/*
*/
    }

    private double getSlideSoftLimit() {
       return RobotConstants.EL_MAX_ENCODER;
    }
    private double getArmSoftLimit() {
        return ARM_MAX_ENCODER;
    }

    private void controlSlides()
    {

        double lftPwr =gpad2.value(ManagedGamepad.AnalogInput.R_TRIGGER_VAL) -gpad2.value(ManagedGamepad.AnalogInput.L_TRIGGER_VAL);
        robot.slides.setLiftSpd(lftPwr, getSlideSoftLimit());
            //robot.elev.moveToCnt(robot.elev.getCurEnc(), RobotConstants.EL_SPD);
        }





    private Pose2d tempPose = new Pose2d();
    private int goLeft = 0;
    private int goRight = 0;
    private int goBrd = 0;
    private double brdDis;

    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    double  turnA            = 0;        // Desired turning power/speed (-1 to +1)

    private void controlDrive()
    {


        if (robot.motors.size() == 0) return;

        raw_lr =  gpad1.value(ManagedGamepad.AnalogInput.R_STICK_X);
        raw_fb = -gpad1.value(ManagedGamepad.AnalogInput.R_STICK_Y);
        raw_turn =  gpad1.value(ManagedGamepad.AnalogInput.L_STICK_X);

        //boolean strt =  gpad1.pressed(ManagedGamepad.Button.START);
        boolean  goto4Tag = gpad1.just_pressed(ManagedGamepad.Button.Y);
        boolean incr = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
        boolean decr = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
        boolean intakeOn = gpad1.pressed(ManagedGamepad.Button.L_BUMP);
        boolean intakeRev = gpad1.pressed(ManagedGamepad.Button.R_BUMP);
        boolean hspd = gpad1.pressed(ManagedGamepad.Button.R_TRIGGER);
        boolean slow = gpad1.pressed(ManagedGamepad.Button.L_TRIGGER);
        boolean dtrn = gpad1.pressed(ManagedGamepad.Button.X);
//        boolean tglF = gpad1.just_pressed(ManagedGamepad.Button.Y);
        boolean drvbrd = gpad1.just_pressed(ManagedGamepad.Button.A) && !gpad1.pressed(ManagedGamepad.Button.START);
        boolean rightOne = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);
        boolean leftOne = gpad1.just_pressed(ManagedGamepad.Button.D_LEFT);
        boolean spin = gpad1.just_pressed(ManagedGamepad.Button.B);
        boolean spinBasket = gpad1.just_pressed(ManagedGamepad.Button.X);

        boolean goto2 = gpad1.just_pressed(ManagedGamepad.Button.A) && !gpad1.pressed(ManagedGamepad.Button.START);
        boolean goto3 = gpad1.just_pressed(ManagedGamepad.Button.B) && !gpad1.pressed(ManagedGamepad.Button.START);

        if (intakeOn)
        {
            robot.crAzYIntake.setPwr(1);
        }
        else if (intakeRev)
        {
            robot.crAzYIntake.setPwr(-1);
        }
        else robot.crAzYIntake.setPwr(0);

//        boolean algn = gpad1.just_pressed(ManagedGamepad.Button.A) && !strt;
//        boolean strf = gpad1.just_pressed(ManagedGamepad.Button.B) && !strt;
//        boolean auto = gpad1.just_pressed(ManagedGamepad.Button.X);
//
//        if(algn || strf || auto)
//        {
//            if(autoDriveActive)
//            {
//                mechDrv.cancelFollowing();
//                autoDriveActive = false;
//                mechDrv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                autoDriveActive = false;
//            }
//            else
//            {
//                mechDrv.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                if(algn)
//                {
//                    int sh = 1;
//                    if(robot.getAlliance() == Field.Alliance.BLUE) sh = -1;
//
//                    double dstAng = sh * Math.PI/2.0;
//                    double curAng = poseEstimate.getHeading();
//
//                    RobotLog.dd(TAG, "Auto turning from %.2f to %.2f", curAng, dstAng);
//
//                    mechDrv.turnAsync(dstAng - curAng);
//                    autoDriveActive = true;
//                }
//
//                if(strf)
//                {
//                    int sh = 1;
//                    if(robot.getAlliance() == Field.Alliance.BLUE) sh = -1;
//
//                    Pose2d dstPose = new Pose2d(Field.halfField - RobotConstants.BOT_WID/2.0 -0.5,
//                        poseEstimate.getY(), sh * Math.PI/2.0);
//                    Pose2d curPose = poseEstimate;
//
//                    RobotLog.dd(TAG, "Auto strafing from %s to %s", curPose, dstPose);
//
//                    TrajectorySequenceBuilder tsb = new TrajectorySequenceBuilder(
//                        poseEstimate, poseEstimate.getHeading(),
//                        RobotConstants.defVelConstraint, RobotConstants.defAccelConstraint,
//                        RobotConstants.MAX_ANG_VEL, RobotConstants.MAX_ANG_ACCEL);
//
//                    tsb.lineToLinearHeading(dstPose);
//
//                    TrajectorySequence tsq = tsb.build();
//
//                    mechDrv.followTrajectorySequenceAsync(tsq);
//
//                    autoDriveActive = true;
//                }
//
//                if(auto)
//                {
//                    TrajectorySequence tsq = route.stateTrajMap.get(Route.State.DROP0);
//
//                    RobotLog.dd(TAG, "Auto driving/dumping");
//                    mechDrv.followTrajectorySequenceAsync(tsq);
//
//                    autoDriveActive = true;
//                }
//            }
//        }
//        else
//        {
//            if (autoDriveActive) return;
//        }

//        if (tglF) useField = !useField;       //We stole this button for the AprilTag detection and so commented this out (also, drivers were not using it at all this year)

        lr = ishaper.shape(raw_lr, 0.02);
        fb = ishaper.shape(raw_fb, 0.02);
        turn = ishaper.shape(raw_turn, 0.02);

        if      (incr) dSpd += dStp;
        else if (decr) dSpd -= dStp;
        dSpd = Range.clip(dSpd, 0.0, 1.0);





        Vector2d driveInput;
        if(useField)
        {
            // Rotate input vector by the inverse of current bot heading
            driveInput = new Vector2d(lr, fb).rotated(-poseEstimate.getHeading());
        }
        else
        {
            driveInput = new Vector2d(fb, -lr);
        }

        double maxCPS = RobotConstants.DT_SAF_CPS;
        if(hspd) maxCPS = RobotConstants.DT_MAX_CPS;
        if (slow) maxCPS = RobotConstants.DT_SAF_CPS/3;
        double spdScl = maxCPS/RobotConstants.DT_MAX_CPS;

        driveInput = driveInput.times(spdScl);
        turn = turn * spdScl;
        Pose2d velPose;
        Timer timer = new Timer();
        if(Math.abs(lr)+Math.abs(fb) > .25){        //if joystick is being used
           clearDriverOveride();
        }








/*
        if (rightOne){
            goRight = 1;
            goLeft = 0;
            TimerTask task = new TimerTask() {
                @Override
                public void run() {
                    clearDriverOveride();
                }
            };
            timer.schedule(task, 240);
        }else if (leftOne){
            goLeft = 1;
            goRight = 0;
            TimerTask task = new TimerTask() {
                @Override
                public void run() {
                    clearDriverOveride();
                }
            };
            timer.schedule(task, 240);
        }else if(goto1){
            TimerTask task = new TimerTask() {
                @Override
                public void run() {
                    goLeft = 0;
                }
            };
            timer.schedule(task, 240);
            goLeft = 1;
            goBrd = goBrd == 0?2:0;
            brdDis = getBrdDis();   //not for April tags, for distance sensor
        }else if(goto2){
            goBrd = goBrd == 0?2:0;
            brdDis = getBrdDis();   //not for April tags, for distance sensor
        }else if(goto3){       //mod of 3
            TimerTask task = new TimerTask() {
                @Override
                public void run() {
                    goRight = 0;
                }
            };
            timer.schedule(task, 480);
            goRight = 1;
            goBrd = goBrd == 0?2:0;
            brdDis = getBrdDis();   //not for April tags, for distance sensor
        }else*/


        RobotLog.dd(TAG, String.format("goLeft: %d, goRight: %d, goBrd: %d, targetFound: %d", goLeft, goRight, goBrd, targetFound?1:0));

//        if(goLeft == 1 || goRight == 1 || goBrd == 2){
//                velPose = new Pose2d(-BORD_SPD*goBrd, BUTT_SPD*goRight -BUTT_SPD*goLeft, Math.toRadians(0));
//        } else

            velPose = new Pose2d(driveInput, -turn);
            mechDrv.setWeightedDrivePower(velPose);

    }

    private void clearDriverOveride(){
        mechDrv.cancelFollowing();
        autoDriveActive=false;
        goRight = 0;
        goLeft = 0;
        goBrd = 0;
    }






//    package org.firstinspires.ftc.teamcode.test;



    /*
     * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
     * The code assumes a Holonomic (Mecanum or X Drive) Robot.
     *
     * For an introduction to AprilTags, see the ftc-docs link below:
     * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
     *
     * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
     * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
     * https://ftc-docs.firstinspires.org/apriltag-detection-values
     *
     * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
     * driving towards the tag to achieve the desired distance.
     * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
     * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
     *
     * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
     * The motor directions must be set so a positive power goes forward on all wheels.
     * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
     * so you should choose to approach a valid tag ID (usually starting at 0)
     *
     * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
     * Manually drive the robot until it displays Target data on the Driver Station.
     *
     * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
     * Release the Left Bumper to return to manual driving mode.
     *
     * Under "Drive To Target" mode, the robot has three goals:
     * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
     * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
     * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
     *
     * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
     * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
     *
     * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
     * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
     *
     */


    /*********************** copied from robotAutoDriveToAprilTag2023 ***********************/
        // Adjust these numbers to suit your robot.
        final double DESIRED_DISTANCE = 13.0; //  this is how close the camera should get to the target (inches)
        double DESIRED_HEADING = 0; 
        final double DESIRED_YAW = -5;

        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double STRAFE_GAIN =  0.04 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        final double MAX_AUTO_SPEED = .1;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_STRAFE= 1;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN  = 0.1;   //  Clip the turn speed to this max value (adjust for your robot)

        private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
        private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
        private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
        private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

        private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
        private static int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
        private VisionPortal visionPortal;               // Used to manage the video source.
        private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
        private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

//    {
//            boolean targetFound     = false;    // Set to true when an AprilTag target is detected
//            double  drive           = 0;        // Desired forward power/speed (-1 to +1)
//            double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
//            double  turn            = 0;        // Desired turning power/speed (-1 to +1)
//
//            // Initialize the Apriltag Detection process
//            initAprilTag();
//            boolean goBrd = false;




//            if (USE_WEBCAM)
//                setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

            // Wait for driver to press start


//            while (opModeIsActive())
//            {
//                targetFound = false;
//                desiredTag  = null;
//
//                gpad1.update();
//                boolean goto1 = gpad1.just_pressed(ManagedGamepad.Button.X);
//                boolean goto2 = gpad1.just_pressed(ManagedGamepad.Button.A);
//                boolean goto3 = gpad1.just_pressed(ManagedGamepad.Button.B);
//
//                if(goto1){
//                    DESIRED_TAG_ID = 1;
//                    goBrd = !goBrd;
//                }
//                if(goto2){
//                    DESIRED_TAG_ID = 2;
//                    goBrd = !goBrd;
//                }
//                if(goto3){
//                    DESIRED_TAG_ID = 3;
//                    goBrd = !goBrd;
//                }
//
//
//                // Step through the list of detected tags and look for a matching tag
//                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//                for (AprilTagDetection detection : currentDetections) {
//                    // Look to see if we have size info on this tag.
//                    if (detection.metadata != null) {
//                        //  Check to see if we want to track towards this tag.
//                        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
//                            // Yes, we want to use this tag.
//                            targetFound = true;
//                            desiredTag = detection;
//                            break;  // don't look any further.
//                        } else {
//                            // This tag is in the library, but we do not want to track it right now.
//                            telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
//                        }
//                    } else {
//                        // This tag is NOT in the library, so we don't have enough information to track to it.
//                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
//                    }
//                }
//
//                // Tell the driver what we see, and what to do.
//                if (targetFound) {
//                    telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
//                    telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
//                    telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
//                    telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
//                    telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
//                } else {
//                    telemetry.addData("\n>","Drive using joysticks to find valid target\n");
//                }
//                telemetry.addData("Searching for", "%d", DESIRED_TAG_ID);
//
//
//                // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
//                if ((gamepad1.left_bumper || goBrd) && targetFound) {
//
//                    // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
//                    double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
//                    double  headingError    = desiredTag.ftcPose.bearing;
//                    double  yawError        = desiredTag.ftcPose.yaw;
//
//                    // Use the speed and turn "gains" to calculate how we want the robot to move.
//                    drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//                    turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
//                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//
//                    telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
//                } else {
//
//                    // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
//                    drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
//                    strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
//                    turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
//                    telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
//                    goBrd = false;
//                }
//                telemetry.update();
//
//                // Apply desired axes motions to the drivetrain.
//                moveRobot(drive, strafe, turn);
//                sleep(10);
//            }
//        }
//
//        /**
//         * Move robot according to desired axes motions
//         * <p>
//         * Positive X is forward
//         * <p>
//         * Positive Y is strafe left
//         * <p>
//         * Positive Yaw is counter-clockwise
//         */

        public void moveRobot(double x, double y, double yaw) {
            // Calculate wheel powers.
            double leftFrontPower    =  x -y -yaw;
            double rightFrontPower   =  x +y +yaw;
            double leftBackPower     =  x +y -yaw;
            double rightBackPower    =  x -y +yaw;

            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send powers to the wheels.
//            leftFrontDrive.setPower(leftFrontPower);
//            rightFrontDrive.setPower(rightFrontPower);
//            leftBackDrive.setPower(leftBackPower);
//            rightBackDrive.setPower(rightBackPower);
            mechDrv.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);
        }

        /**
         * Initialize the AprilTag processor.
         */
        private void initAprilTag() {
            // Create the AprilTag processor by using a builder.
            aprilTag = new AprilTagProcessor.Builder().build();

            // Adjust Image Decimation to trade-off detection-range for detection-rate.
            // eg: Some typical detection data using a Logitech C920 WebCam
            // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
            // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
            // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
            // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
            // Note: Decimation can be changed on-the-fly to adapt during a match.
            aprilTag.setDecimation(2);

            // Create the vision portal by using a builder.
            if (USE_WEBCAM) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "webcamRIGHT"))
                        .addProcessor(aprilTag)
                        .build();
                setManualExposure(5, 250);  // Use low exposure time to reduce motion blur
            } else {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(BuiltinCameraDirection.BACK)
                        .addProcessor(aprilTag)
                        .build();
            }
        }

        /*
         Manually set the camera gain and exposure.
         This can only be called AFTER calling initAprilTag(), and only works for Webcams;
        */
        private void    setManualExposure(int exposureMS, int gain) {
            // Wait for the camera to be open, then use the controls

            if (visionPortal == null) {
                return;
            }

            // Make sure camera is streaming before we try to set the exposure controls
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Camera", "Waiting");
                telemetry.update();
                while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    sleep(20);
                }
                telemetry.addData("Camera", "Ready");
                telemetry.update();
            }

            // Set camera controls unless we are stopping.
            if (!isStopRequested())
            {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
                sleep(20);
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(gain);
                if(VERBOSE){RobotLog.dd(TAG, String.format("Min Gain: %d, Max Gain: %d", gainControl.getMinGain(), gainControl.getMaxGain()));}
                sleep(20);
            }
        }


    private Pose2d setEndPose(Pose2d p1, double y){
        return new Pose2d(p1.getX(), p1.getY() + y, p1.getHeading());
    }
    private boolean comparePoses(Pose2d p1, Pose2d p2)
    {
        boolean similar = true;
        if(Math.abs(p1.getX()-p2.getX()) > POSE_EQUAL){
            similar = false;
        }
        if(Math.abs(p1.getY()-p2.getY()) > POSE_EQUAL){
            similar = false;
        }
        if(Math.abs(Math.toDegrees(p1.getHeading())-Math.toDegrees(p2.getHeading()))/5 > POSE_EQUAL){
            similar = false;
        }
        return similar;
    }

    double spinTime;
    double liftTime;
    double intTime;
    double drvTime;
    double u, c, d, p, L, f, w;
    private double END_GAME_TIMEOUT = 85;
    private boolean inEndGamePeriod = false;
    private boolean enabledDroneLaunching = true;
    private final ElapsedTime endGameNotificationRumble = new ElapsedTime();
    private final ElapsedTime oTimer = new ElapsedTime();
    private final ElapsedTime opTimer = new ElapsedTime();

    private boolean lBumperPressed;


    private void processControllerInputs()
    {


        gpad2.update();
        boolean dpadUp= gpad2.pressed(ManagedGamepad.Button.D_UP);
        boolean dpadLeft= gpad2.pressed(ManagedGamepad.Button.D_LEFT);
        boolean dpadRight= gpad2.pressed(ManagedGamepad.Button.D_RIGHT);
        boolean dpadDown= gpad2.pressed(ManagedGamepad.Button.D_DOWN);

        if (dpadDown ||dpadUp||dpadRight||dpadLeft){
            robot.shooter.setDistance(1);
        }
        else {
            robot.shooter.stop();
        }

        /* Rumble in the last 10 sec of match with custom effect */
//        if (endGameNotificationRumble.seconds() > END_GAME_TIMEOUT)
//        {
//            gpad2.customRumble(1);
//            gpad1.customRumble(1);
//            /* Sequence matters Processing Driver inputs gets called after
//            * This is where the timer maintenance will be taken care of
//            * */
//            endGameNotificationRumble.reset();
//            setENDGAMETIMOUTSECS(25.0);
//            inEndGamePeriod = true;
//        }
//
//        /* Rumble in the last 5 sec of match */
//        if (
//                (endGameNotificationRumble.seconds() > END_GAME_TIMEOUT) &&
//                (inEndGamePeriod == true)
//           )
//        {
//            gpad2.customRumble(2);
//            gpad1.customRumble(2);
//            endGameNotificationRumble.reset();
//            inEndGamePeriod = false;
//        }

//        robot.setElbowMotor(gpad2.value(ManagedGamepad.AnalogInput.L_STICK_Y));
//        robot.setExtenderPower(gpad2.value(ManagedGamepad.AnalogInput.R_TRIGGER_VAL)-gpad2.value(ManagedGamepad.AnalogInput.L_TRIGGER_VAL));
//        dashboard.displayText(11, String.format(Locale.US, " LEFT TRIGGER(%f) - RIGHT TRIGGER(%f) = %f",gpad2.value(ManagedGamepad.AnalogInput.R_TRIGGER_VAL),gpad2.value(ManagedGamepad.AnalogInput.L_TRIGGER_VAL),gpad2.value(ManagedGamepad.AnalogInput.R_TRIGGER_VAL)-gpad2.value(ManagedGamepad.AnalogInput.L_TRIGGER_VAL)));

//        robot.elbowMotor.moveToLevel();



 /*       if(gpad2.just_pressed(ManagedGamepad.Button.L_BUMP))
        {
            robot.toggleBucketServoForward();
            robot.pixelPieces --;
            robot.pixelPieces --;
            RobotLog.dd(TAG, "pixel detected: %d, pieces: %d", pixelDetected?1:0, robot.pixelPieces);
        }
        if (robot.pixelPieces < 0){
            robot.pixelPieces = 0;
            RobotLog.dd(TAG, "pixel detected: %d, pieces: %d", pixelDetected?1:0, robot.pixelPieces);
        }

        if(gpad2.just_pressed(ManagedGamepad.Button.R_BUMP))
        {
            robot.toggleBucketServoBackward();
        }

        if(gpad2.just_pressed(ManagedGamepad.Button.X))
        {
            robot.destendr();
            robot.toggleIntakes();
            if(robot.rearDistSensor.getDistance(DistanceUnit.CM) > 30) {
                robot.elbowMotor.moveToLevel(0, EL_SPD_DWN);
                stackLvlCnt = 0;
            }

        }
        if(gpad2.just_pressed(ManagedGamepad.Button.A))
        {
            Timer timer = new Timer();
            if(robot.rearDistSensor.getDistance(DistanceUnit.CM) > 30 || robot.elbowMotor.getCurEnc() > -800 ) {
                robot.destendr();
                if (robot.extenderMotor.getCurEnc() < 100) {
                    robot.elbowMotor.moveToLevel(1, EL_SPD);
                    stackLvlCnt = 1;
                } else{
                    robot.elbowMotor.moveToLevel(1, 0.35);
                    stackLvlCnt = 1;
                }
            }
            robot.sweeperServo1.moveAtRate(-0.5);
            robot.sweeperServo2.moveAtRate(-0.5);
            robot.intakesOn = true;
            TimerTask task = new TimerTask() {
                @Override
                public void run() {
                    robot.intakesOff();
                }
            };
            timer.schedule(task, 500);
        }

        if(gpad2.just_pressed(ManagedGamepad.Button.B) && !gpad2.pressed(ManagedGamepad.Button.START))
        {
            if (!ballfinalclim & !ballfinalclimout) {
			    robot.elbowMotor.moveToCnt(-2400, 1);
                ballfinalclim = true;
            } else if (ballfinalclim & !ballfinalclimout & robot.elbowMotor.getCurEnc() < -2300) {
                robot.extendr(1);
                ballfinalclimout = true;
            } else if (ballfinalclim & ballfinalclimout) {
                ballfinalclim = false;
                ballfinalclimout = false;
  
                robot.destendr();
            } else {
                robot.elbowMotor.moveToCnt(-2400, 1);
                ballfinalclim = true;

            }
        }
        if(gpad2.pressed(ManagedGamepad.Button.START) && (gpad2.just_pressed(ManagedGamepad.Button.L_BUMP) || gpad2.just_pressed(ManagedGamepad.Button.R_BUMP)))
        {
            robot.wristServo.init(-100,100);
            wristRestrictionsOff = true;
        }
        /* disable triangele button during Teleop to prevent accidental Drone Deployment */

       if(gpad2.just_pressed(ManagedGamepad.Button.D_DOWN)){
           robot.currentMode= MecanumBot.ElbowPositions.SAMPLE_MODE;
           robot.armLevelDown(); //move arm and slides down

        }

        if(gpad2.just_pressed(ManagedGamepad.Button.D_UP)){
            robot.currentMode= MecanumBot.ElbowPositions.SAMPLE_MODE;
            robot.armLevelUp(); //move arm and slides up

        }


        if(gpad2.just_pressed(ManagedGamepad.Button.B) && !gpad2.pressed(ManagedGamepad.Button.START)) {
            robot.drivePos(); //move arm and slides up
        }


        if(gpad2.just_pressed(ManagedGamepad.Button.A) && !gpad2.pressed(ManagedGamepad.Button.START)) {
            robot.pickUp(); //move arm and slides up
        }




        controlArm();

        controlSlides();


        //presetClaws();

        //armNslidesLevs();


        opTimer.reset();
        liftTime = opTimer.milliseconds();
        opTimer.reset();
        spinTime = 0;
        controlIntake();

        intTime = opTimer.milliseconds();
        opTimer.reset();
    }

    private void processDriverInputs()
    {
        gpad1.update();
        boolean foundUp = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);
        boolean foundDown = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);

        if(foundUp) {

            robot.releaseFound();
        }
        if(foundDown) {

            robot.clampFound();
        }
        controlDrive();
        drvTime = opTimer.milliseconds();
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this);
        RobotLog.dd(TAG,"RUNNING INIT IN MecanumTeleop");

        initPreStart();

        dashboard.displayText(0, robot.getName() + " is ready");

        // Wait for the game to start (driver presses PLAY)
        while(!isStarted() && !isStopRequested())
        {
            update();
            if(VERBOSE) { printTelem();}
            doLogging(0);
            robot.finishFrame();
            robot.waitForTick(20);
        }

        RobotLog.dd(TAG, "Mecanum_Driver starting");

        opTimer.reset();
        endGameNotificationRumble.reset();
        
        while (opModeIsActive())
        {
            oTimer.reset();
            update();
            u=opTimer.milliseconds();
//            oTimer.reset();
            processControllerInputs();
            c=opTimer.milliseconds();
            processSensors();
//            oTimer.reset();
            processDriverInputs();
            d=opTimer.milliseconds();
            processSensors();
//            oTimer.reset();
//            printTelem();
//            p=opTimer.milliseconds();
//            oTimer.reset();
            doLogging(oTimer.milliseconds());
//            L=opTimer.milliseconds();
//            oTimer.reset();
//            robot.finishFrame();
//            f=opTimer.milliseconds();
//            oTimer.reset();
//            robot.waitForTick(20);
            w=opTimer.milliseconds();
//            oTimer.reset();
//            if(VERBOSE){RobotLog.vv(TAG, "looping");}
        }
    }

    double spinnerPwr = RobotConstants.SP_POWER;
    double moveAtRate = 0.0;
    private double lastMrkPos = RobotConstants.MK_ARM_STOW;

    double dSpd = 0.0;
    double dStp = 0.1;

    static final double spdScl = Math.sqrt(2.0);
    Input_Shaper ishaper = new Input_Shaper();

    private boolean useField = false;
    private final MecanumBot robot = new MecanumBot();
    private MecanumDriveLRR  mechDrv;

    double raw_lr;
    double raw_fb;
    double raw_turn;
    double lr;
    double fb;
    double turn;

    Pose2d poseEstimate;

    int[] cnts = {0,0,0,0};
    double[] vels = {0,0,0,0};
    double hdg = 0;

    private String lStr = "";
    private String aStr ="";
    private int l = 0;

    boolean wristRestrictionsOff = false;
    boolean ballfinalclim = false;
    boolean ballfinalclimout = false;
    private int numElem = 0;

    private SpinRoute spinRoute;
    private SpinRoutebasket spinRoutebasket;


    private TouchSensor armButton;

    //private PPlayRoute route;
    boolean autoDriveActive = false;

    public void setENDGAMETIMOUTSECS(double endGameTimeOutTime)
    {
        this.END_GAME_TIMEOUT = endGameTimeOutTime;
    }
}
