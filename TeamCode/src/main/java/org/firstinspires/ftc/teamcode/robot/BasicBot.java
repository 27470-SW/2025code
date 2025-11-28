
package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.field.Field;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.ImuRunner;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Units;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a BasicBot
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 */
@SuppressWarnings({"WeakerAccess", "FieldCanBeLocal", "unused"})
public class BasicBot
{
    private static final String TAG = "SJH_BOT";

    protected LinearOpMode op = null;
    public static RobotConstants rbc; // = new RobotConstants();
    protected CommonUtil cmu = CommonUtil.getInstance();
    protected HardwareMap hwMap = null;

    public enum OpModeType {TELE, AUTO, UNKNOWN}
    public static OpModeType curOpModeType = OpModeType.UNKNOWN;

    protected List<LynxModule> allHubs = null;
    protected LynxModule.BulkCachingMode bulkCachingMode =  LynxModule.BulkCachingMode.AUTO;

    /* Public OpMode members. */

    public Drive drive = null;
    public DcMotorEx  leftMotor   = null;
    public DcMotorEx  rightMotor  = null;

    public List<DcMotorEx> leftMotors  = new ArrayList<>(2);
    public List<DcMotorEx> rightMotors = new ArrayList<>(2);

    public int numLmotors = 0;
    public int numRmotors = 0;
    public Map<String, DcMotorEx> motors = new HashMap<>();

    public NormalizedColorSensor colorSensor = null;
    public DistanceSensor rearDistSensor = null;
    public ColorSensor pixelColorSensor = null;
    private int clrR = -1;
    private int clrG = -1;
    private int clrB = -1;
    private NormalizedRGBA clrRGBA;
    private final int colorPort = 0;
    boolean colorEnabled = false;


    public IMU imu = null;
    public boolean gyroInverted = true;
    public boolean initDirSensor = true;
    private Orientation cachedAngles;
    private ImuRunner imuRunner;
    private final ElapsedTime botTimer = new ElapsedTime();
    private final boolean useImuThread = false;
    public boolean gyroReady = false;

    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    private final int[] cnts = {0,0,0,0};
    private final double[] vels = {0,0,0,0};
    private double hdg = 0;

    private long updateFrame = 0;
    private long gyroFrame = 0;

    //Distance from ctr of rear wheel to tail
    public float REAR_OFFSET;
    public float FRNT_OFFSET;
    protected static float CAMERA_X_IN_BOT;
    protected static float CAMERA_Y_IN_BOT;
    protected static float CAMERA_Z_IN_BOT;

    private final DriveDir defDriveDir = DriveDir.PUSHER;
    private DriveDir ddir = defDriveDir;
    private boolean invertDrive = false;
    public DriveDir calibrationDriveDir = DriveDir.UNKNOWN;

    protected String name = "BasicBot";
    public RobotConstants.Chassis chassis = RobotConstants.Chassis.B7253;

    private static Field.Alliance alliance = Field.Alliance.RED;

    private int initHdg = 0;
    public static double autonEndHdg = 0.0;
    public double getAutonEndHdg() {return  autonEndHdg;}
    public void setAutonEndHdg(double hdg) {autonEndHdg = hdg;}
    public static Point2d autonEndPos = new Point2d("AEND", 0.0, 0.0);
    public Point2d getAutonEndPos() {return autonEndPos;}
    public void setAutonEndPos(Point2d endPos) {autonEndPos = endPos;}
    public Field.Alliance getAlliance() {return alliance;}
    public void setAlliance(Field.Alliance alnc) {alliance = alnc;}

    public static DcMotor.Direction  LEFT_DIR = DcMotor.Direction.FORWARD;
    public static DcMotor.Direction RIGHT_DIR = DcMotor.Direction.REVERSE;

    public float BOT_LENGTH;

    protected double COUNTS_PER_MOTOR_REV;
    protected double[] DRIVE_GEARS;

    public double WHEEL_DIAMETER_INCHES;
    protected double TUNE;
    public double CPI;

    Map<String, Boolean> capMap = new HashMap<>();

    private final ElapsedTime period  = new ElapsedTime();

    protected static final boolean VERBOSE = RobotConstants.logVerbose;

    /* Constructor */
    public BasicBot()
    {
        //Neverest classic 20,40,60, and orbital 20 have 7 rising edges of Channel A per revolution
        //with a quadrature encoder (4 total edges - A rise, B rise, A fall, B fall) for a total
        //of 28 counts per pre-gear box motor shaft revolution.
        COUNTS_PER_MOTOR_REV = 28;//RobotConstants.//28;
        DRIVE_GEARS = new double[]{40.0, 1.0/2.0};

        WHEEL_DIAMETER_INCHES = 4.1875;
        TUNE = 1.00;

        BOT_LENGTH = (float)RobotConstants.BOT_LEN;

        REAR_OFFSET = BOT_LENGTH/2.0f;
        FRNT_OFFSET = BOT_LENGTH - REAR_OFFSET;

        CAMERA_X_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;
        CAMERA_Y_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;
        CAMERA_Z_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;

        capMap.put("drivetrain", false);
        capMap.put("shooter",    false);
        capMap.put("collector",  false);
        capMap.put("pusher",     false);
        capMap.put("sensor",     false);
        capMap.put("arm",        false);
        capMap.put("holder",     false);
        capMap.put("lifter",     false);
    }

    public void initCore(LinearOpMode op)
    {
        computeCPI();
        initOp(op);
        allHubs = hwMap.getAll(LynxModule.class);
        setBcm(bulkCachingMode);
    }

    /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode op, boolean initDirSensor)
    {
        RobotLog.dd(TAG, "BasicBot init");
        initCore(op);

        this.initDirSensor = initDirSensor;
        initSensors();
        initDriveMotors();
        initJointControl();
        initCapabilities();
    }

    public void init(LinearOpMode op)
    {
        init(op, true);
    }

    public void init(LinearOpMode op, RobotConstants.Chassis chassis, boolean initDirSensor)
    {
        this.chassis = chassis;
        this.name = chassis.name();
        init(op, initDirSensor);
    }

    protected void initOp(LinearOpMode op)
    {
        RobotLog.dd(TAG, "BasicBot initOp");
        this.op = op;
        this.hwMap = op.hardwareMap;
    }

    protected void initDriveMotors()
    {
        RobotLog.dd(TAG, "BasicBot initDriveMotors");
        // FORWARD for CCW drive shaft rotation if using AndyMark motors
        // REVERSE for  CW drive shaft rotation if using AndyMark motors
        try  //Drivetrain
        {
            leftMotor  = hwMap.get(DcMotorEx.class,"leftdrive");
            rightMotor = hwMap.get(DcMotorEx.class,"rightdrive");

            leftMotors.add(numLmotors++, leftMotor);
            rightMotors.add(numRmotors++, rightMotor);

            leftMotor.setDirection(LEFT_DIR);
            rightMotor.setDirection(RIGHT_DIR);
            motors.put("FL", leftMotor);
            motors.put("FR", rightMotor);

            DcMotorEx lm0 = leftMotors.get(0);
            PIDFCoefficients pid;
            pid = lm0.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            RobotLog.dd(TAG, "RUN_TO_POS Motor PIDs. P:%.2f I:%.2f D:%.2f F:%.2f",
                    pid.p, pid.i, pid.d, pid.f);
            pid = lm0.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            RobotLog.dd(TAG, "RUN_USING_ENC Motor PIDs. P:%.2f I:%.2f D:%.2f F:%.2f",
                    pid.p, pid.i, pid.d, pid.f);

            for (DcMotorEx m : leftMotors)
            {
                PIDFCoefficients lpid;
                lpid = m.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
                lpid.p = 6.0;
                m.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, lpid);
            }

            for (DcMotorEx m : rightMotors)
            {
                PIDFCoefficients rpid;
                rpid = m.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
                rpid.p = 6.0;
                m.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, rpid);
            }

            capMap.put("drivetrain", true);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initDriveTrain\n" + e.toString());
        }

        for(DcMotor mot : motors.values())
        {
            if(mot != null)
            {
                mot.setPower(0);
                mot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    protected void initJointControl()
    {
        RobotLog.dd(TAG, "BasicBot collector/lifter - empty");
    }


    protected void initPushers()
    {
        RobotLog.dd(TAG, "BasicBot pushers - empty");
    }

    public boolean initImu()
    {
        boolean imuGood = false;
        RobotLog.dd(TAG, "In BasicBot.initSensors");
//        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMU_IMUCalibration.json";
//        parameters.loggingEnabled      = false;
//        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // If hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        try
        {
            imu = hwMap.get(IMU.class, "imu");
            if(initDirSensor)
            {
                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));
                imu.initialize(parameters);
                RobotLog.dd(TAG, "In BasicBot Initializing imu on Control HUB");
//                AxesOrder order = AxesOrder.ZYX;
//                AxesSigns signs = AxesSigns.PPP;
//                int[] result = order.indices();
//
//                BNO055IMUUtil.remapAxes(imu, order, signs);
//                BNO055IMUUtil.remapZAxis(imu, AxisDirection.POS_Y);

            }

//            if(useImuThread)
//            {
//                imuRunner = ImuRunner.getInstance(imu);
//                imuRunner.start();
//            }

            imuGood = true;
        }
        catch(Exception e)
        {
            RobotLog.ee(TAG, "ERROR get imu\n" + e.toString());
        }

        return imuGood;
    }

    boolean imuGood;
    protected void initSensors()
    {
        imuGood = initImu();

    }

    protected void initCapabilities()
    {
        for (Map.Entry<String, Boolean> mEnt : capMap.entrySet())
        {
            RobotLog.dd(TAG, mEnt.getKey() + " = " + mEnt.getValue());
        }
    }

    public boolean getCapability(String cap)
    {
        Boolean isCap = capMap.get(cap);
        return isCap != null && isCap;
    }

    private double getTotalGearRatio()
    {
        double gr = 1.0;
        for(double g : DRIVE_GEARS) gr *= g;
        return gr;
    }

    public double getBatteryVoltage()
    {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hwMap.voltageSensor)
        {
            double voltage = sensor.getVoltage();
            if (voltage > 0)
            {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public int getClrR() {return clrR;}
    public int getClrG() {return clrG;}
    public int getClrB() {return clrB;}
    public float getClrV()
    {
        float[] hsv = new float[3];
        Color.RGBToHSV(clrR, clrB, clrG, hsv);
        return hsv[2];
    }

    protected void computeCPI()
    {
        CPI = RobotConstants.DT_CPI;
    }

    public void setBcm(LynxModule.BulkCachingMode bcm)
    {
        for (LynxModule module : allHubs)
        {
            module.setBulkCachingMode(bcm);
        }
    }

    public void setDriveDir (DriveDir ddir)
    {
        RobotLog.ii(TAG, "setDriveDir to " + ddir + " from " + this.ddir);
        if(ddir == DriveDir.UNKNOWN)
        {
            RobotLog.ee(TAG, "setDriveDir called with UNKNOWN");
            return;
        }

        if(ddir == DriveDir.RIGHT || ddir == DriveDir.LEFT)
        {
            RobotLog.dd(TAG, "setDriveDir called with R/L");
            if(calibrationDriveDir == DriveDir.UNKNOWN) calibrationDriveDir =
                RobotConstants.DT_DIR;
            return;
        }

        if(calibrationDriveDir == DriveDir.UNKNOWN) calibrationDriveDir = ddir;

        if(this.ddir == ddir)
            return;

        RobotLog.ii(TAG, "Setting Drive Direction to " + ddir + " from " + this.ddir);
        RobotLog.ii(TAG, "Calibration Drive Direction: " + calibrationDriveDir);

        this.ddir = ddir;

        invertDrive = ddir != calibrationDriveDir;
    }

    public void setInitHdg(double initHdg)
    {
        this.initHdg = (int) Math.round(initHdg);
    }

    public boolean calibrateGyro()
    {
        //Callibration performed internally and assisted by loading file from offline calib.
//        RobotLog.dd(TAG, "CalibrateGyro isAccelCal %s isGyroCal %s calStatus %s sysStatus %s",
//            imu.isAccelerometerCalibrated(),
//            imu.isGyroCalibrated(),
//            imu.getCalibrationStatus(),
//            imu.getSystemStatus());

        if(calibrationDriveDir == DriveDir.UNKNOWN)
        {
            RobotLog.ii(TAG, "calibrateGyro called without having set a drive Direction. " +
                "Defaulting to " + RobotConstants.DT_DIR);
            setDriveDir(RobotConstants.DT_DIR);
        }
        RobotLog.ii(TAG, "Calibration drive dir = %s", calibrationDriveDir);
        gyroReady = true;
        return true;
    }

    public void resetGyro()
    {
    }

    public double getGyroHdgDeg()
    {
        return Math.toDegrees(getGyroAngles().firstAngle);
    }

    public Orientation getGyroAngles()
    {
        Orientation rtrnAngles = cachedAngles;
        if(updateFrame != gyroFrame || cachedAngles == null)
        {
            gyroFrame = updateFrame;
//            if(useImuThread) rtrnAngles = imuRunner.getOrientation();
//            else             rtrnAngles = imu.getAngularOrientation();
        }

        return rtrnAngles;
    }

    public AngularVelocity getGyroVelocity()
    {
        return imu.getRobotAngularVelocity(AngleUnit.DEGREES);
    }

//    public double getGyroFhdg()
//    {
//        if(imu == null) return 0;
//        int dirHdgAdj = 0;
//        //if(ddir != calibrationDriveDir) dirHdgAdj = 180;
//
//        double rawGyroHdg = getGyroHdgDeg();
//        //There have been cases where gyro incorrectly returns 0 for a frame : needs filter
//        //Uncomment the following block for a test filter
////        if(rawGyroHdg == 0 &&
////           Math.abs(lastRawGyroHdg) > 30)
////        {
////            rawGyroHdg = lastRawGyroHdg;
////        }
////        else
////        {
////            lastRawGyroHdg = rawGyroHdg;
////        }
//
//        //NOTE:  gyro.getIntegratedZValue is +ve CCW (left turn)
//        //WHEN the freaking gyro is mounted upright.
//        //Since snowman 2.0 has gyro mounted upside down, we need
//        //to negate the value!!
//        int gDir = 1;
//        if(gyroInverted) gDir = -1;
//        double cHdg = gDir * rawGyroHdg + initHdg + dirHdgAdj;
//
//        while (cHdg <= -180) cHdg += 360;
//        while (cHdg >   180) cHdg -= 360;
//
//        return cHdg;
//    }

    public String getName() {return name;}
    public void setName(String name) {this.name = name;}

    //INTAKE is the intake side and should have configured left motors to left.
    public enum DriveDir
    {
        UNKNOWN,
        INTAKE,
        PUSHER,
        LEFT,
        RIGHT
    }

    @SuppressWarnings("unused")
    private int getColorPort()
    {
        return colorPort;
    }

    public boolean getInvertDrive() { return invertDrive; }

    public void turnColorOn()
    {
/*
        if(colorSensor == null) return;
        RobotLog.ii(TAG, "Turning on colorSensor LED");
        if (colorSensor instanceof SwitchableLight)
        {
            colorEnabled = true;
            ((SwitchableLight)colorSensor).enableLight(false);
        }

 */
    }

    public void turnColorOff()
    {
/*
        if(colorSensor == null) return;
        if (colorSensor instanceof SwitchableLight)
        {
            colorEnabled = false;
            ((SwitchableLight)colorSensor).enableLight(false);
        }

 */
    }
    public double colorFindDistance()
    {
        if(colorSensor == null) return -1;
        if (colorSensor instanceof DistanceSensor) {
            return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        }else return -1;

    }

    public DriveDir getDriveDir() { return ddir; }

    double cbcTime;
    double lrrTime;
    double encTime;
    boolean inInit = false;

    public void setInInit(boolean inInit)
    {
        this.inInit = inInit;
    }

    private final String[] motorTags = {"FL", "BL", "BR", "FR"};

    public void update()
    {
        cmu.setTelemetryPacket(new TelemetryPacket());
        RobotLog.dd(TAG, "pose(13) is now: %s", drive.getLocalizer().getPoseEstimate().toString());

        botTimer.reset();
        if(bulkCachingMode == LynxModule.BulkCachingMode.MANUAL)
        {
            for (LynxModule module : allHubs)
            {
                module.clearBulkCache();
            }
        }
        RobotLog.dd(TAG, "pose(14) is now: %s", drive.getLocalizer().getPoseEstimate().toString());

        cbcTime = botTimer.milliseconds();
        botTimer.reset();

        if (drive != null && !inInit)
        {
            RobotLog.dd(TAG, "pose(15) is now: %s", drive.getLocalizer().getPoseEstimate().toString());
            if (drive instanceof MecanumDriveLRR) {
                ((MecanumDriveLRR) drive).update();
                RobotLog.dd(TAG, "pose(16) is now: %s", drive.getLocalizer().getPoseEstimate().toString());
            }
            else drive.updatePoseEstimate();
        }
        RobotLog.dd(TAG, "pose(17) is now: %s", drive.getLocalizer().getPoseEstimate().toString());

        lrrTime = botTimer.milliseconds();
        botTimer.reset();
        RobotLog.dd(TAG, "pose(18) is now: %s", drive.getLocalizer().getPoseEstimate().toString());

        int c = 0;
        for(String s : motorTags)
        {
            if(motors.containsKey(s))
            {
                DcMotorEx m = motors.get(s);
                cnts[c]   = m.getCurrentPosition();
                vels[c++] = m.getVelocity();
            }
        }
        RobotLog.dd(TAG, "pose(19) is now: %s", drive.getLocalizer().getPoseEstimate().toString());


        encTime = botTimer.milliseconds();
        botTimer.reset();

//        hdg = getGyroFhdg();
        double imuTime = botTimer.milliseconds();
        if(VERBOSE)
        {
            RobotLog.dd(TAG, "UPD CBC:%.2f LRR:%.2f ENC:%.2f IMU:%.2f",
                cbcTime, lrrTime, encTime, imuTime);
        }
        /*
        if(colorSensor != null)
        {
            clrRGBA = colorSensor.getNormalizedColors();
            int clr = clrRGBA.toColor();

            clrR = (clr >> 16) & 0xFF;
            clrG = (clr >>  8) & 0xFF;
            clrB = (clr      ) & 0xFF;
        }
*/
        ++updateFrame;
        RobotLog.dd(TAG, "pose(20) is now: %s", drive.getLocalizer().getPoseEstimate().toString());

    }

    public void finishFrame()
    {
//        TelemetryPacket tp = cmu.getTelemetryPacket();
//        if(tp != null) cmu.getFtcDashboard().sendTelemetryPacket(tp);
    }

    public int[] getCnts()    { return cnts; }
    public double[] getVels() { return vels; }
//    public double getHdg()    { return hdg;  }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs)
    {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            op.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    //Robot coordinate frame is center of bot at bottom of wheels
    //X is positive towards front of robot
    //Y is positive towards left side
    //Z is positive up
    //So, robot placed at center of field facing positive X field axis
    //(to right standing in red alliance station) has field
    //coordinates of 0,0,0 and rotation of 0,0,0.

    //With phone laid flat in portrait mode with screen up:
    //The phone axis is 0,0,0 at Camera (using front camera)
    //X pts to the right side of phone (ZTE volume button edge)
    //Y pts to top of phone (head phone jack edge)
    //Z pts out of camera - initially toward bot up
    //to mount camera on front of bot looking bot fwd,
    //rotate -90 about z, then -90 about x
    //translate 0 in bot x, half bot length in bot y, and ~11" in bot z

    //For webcam,  0,0,0 rotation is with lens looking up,
    //and long axis aligned with robot X, short axis with robot Y.
    //Rotate +90 about robot X to get lens pointing out right side of bot
    //then rotate +90 about robot Z to get lens pointing forward.
    //Then translate to mounting point on bot

    public static OpenGLMatrix phoneOrientation;
    public static OpenGLMatrix phoneLocationOnRobot;
    static
    {
        RobotLog.dd(TAG, "Init static block");
        phoneOrientation = Orientation.getRotationMatrix(
            AxesReference.EXTRINSIC, AxesOrder.XYZ, //ZXY
            AngleUnit.DEGREES, 90, 0, 90);

        phoneLocationOnRobot = OpenGLMatrix
            .translation(CAMERA_X_IN_BOT, CAMERA_Y_IN_BOT, CAMERA_Z_IN_BOT)
            .multiplied(phoneOrientation);
    }
}
