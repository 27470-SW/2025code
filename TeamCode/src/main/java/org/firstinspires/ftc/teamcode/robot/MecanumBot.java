package org.firstinspires.ftc.teamcode.robot;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.ARM_NUM_LEVS;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_LEVS;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_NUM_LEVS;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_SPD;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_SPD_DWN;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EX_MAX;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EX_MIN;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.SLIDECPI;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.WR_SENSE;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.CommonUtil;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class MecanumBot extends BasicBot
{
    private static final String TAG = "SJH_MEC";

    //public Claw claw = null;
    //Still need this for now for backward compatibility

    // public MotorComponent elev;

    // public MotorComponent elbowMotor = null;
    //public MotorComponent extenderMotor = null;
    //public MotorComponent bucketServo = null;
    //public ServoComponent wristServo = null;
    //public ServoComponent droneLauncherServo = null;
    //public MotorComponent sweeperServo1 = null;
    //public MotorComponent sweeperServo2 = null;
    //public Intake intake = null;
public Claw claw =null;
public Lifter slides=null;
public MotorComponent arm=null;
public Intake intake=null;
public Claw foundation1 =null;
public Claw foundation2 =null;

    public double logIntakeCurSpd = 0.0;

    public MecanumBot()
    {
        super();

        name= "27470";
        bulkCachingMode =  LynxModule.BulkCachingMode.MANUAL;

        DRIVE_GEARS =
          new double[]{RobotConstants.DT_MOTOR.getGear(), RobotConstants.DT_EXT_GEAR_RATIO};

        WHEEL_DIAMETER_INCHES = RobotConstants.DT_WHEEL_DIAM;

        gyroInverted = false;
    }

    @Override
    public void init(LinearOpMode op, boolean initDirSensor)
    {
        RobotLog.dd(TAG, "MecanumBot init " + name);
        initCore(op);

        this.initDirSensor = initDirSensor;
        initSensors();
        initDriveMotors();
        initJointControl();
        initCapabilities();
    }

    @Override
    protected void initSensors() {
        super.initSensors();

        boolean clrGood = false;
        boolean dist1Good = false;

//        try
//        {
//            colorSensor = hwMap.get(NormalizedColorSensor.class, "color1");
//            colorSensor.setGain(25.0f);
//            clrGood = true;
//        }
//        catch(Exception e)
//        {
//            RobotLog.ee(TAG, "ERROR get colorSensor\n" + e.toString());
//        }
//        try
//        {
//            rearDistSensor = hwMap.get(DistanceSensor.class, "rearDist");
//            dist1Good = true;
//        }
//        catch(Exception e)
//        {
//            RobotLog.ee(TAG, "ERROR get distanceSensor1\n" + e.toString());
//        }
//
//        try
//        {
//            pixelColorSensor = hwMap.get(ColorSensor.class, "color2");
//            dist1Good = true;
//        }
//        catch(Exception e)
//        {
//            RobotLog.ee(TAG, "ERROR get distanceSensor1\n" + e.toString());
//        }
//
//        capMap.put("sensor", clrGood && super.imuGood && dist1Good);
    }

    @Override
    protected void initDriveMotors()
    {
        RobotLog.dd(TAG, TAG + "Initializing drive motors");
        drive = new MecanumDriveLRR(imu);
        //Roadrunner mech drive starts with FL motor and goes counterclockwise (FL,BL,BR,FR)
        try
        {
            List<DcMotorEx> mtrs = ((MecanumDriveLRR)drive).getMotors();
            motors.put("FL", mtrs.get(0));
            motors.put("BL", mtrs.get(1));
            motors.put("BR", mtrs.get(2));
            motors.put("FR", mtrs.get(3));

            capMap.put("drivetrain", true);
            CommonUtil.getInstance().getDashboard().displayText(6, "TRMB_MOTORS: " + mtrs.size());
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get motors in MecanumBot\n" + e.toString());
        }
    }

    private DigitalChannel redLED1;
    private DigitalChannel greenLED1;
    private DigitalChannel redLED2;
    private DigitalChannel greenLED2;



   public int pixelPieces = 0;

    @Override
    protected void initJointControl()
    {
        RobotLog.dd(TAG, "init Lifter MecanumBot\n");
        //claw = new Claw (hwMap);
        //claw.maxClosed = .85;
        //claw.maxOpen = .35;

        //elbowMotor = new MotorComponent("elbow", hwMap);
        //extenderMotor = new MotorComponent("extend", hwMap);
        //bucketServo = new MotorComponent("bucket", hwMap);
        //wristServo = new ServoComponent("wrist", hwMap);
        //droneLauncherServo = new ServoComponent("drone", hwMap);
        //sweeperServo1 = new MotorComponent("sweeper1", hwMap);
        //sweeperServo2 = new MotorComponent("sweeper2", hwMap);
        //intake = new Intake(hwMap);

        claw = new Claw(hwMap, "clawservo");
        slides = new Lifter( "slide1", "slide2",hwMap);
        arm = new MotorComponent("arm",hwMap);
        intake = new Intake(hwMap);
        foundation1 = new Claw(hwMap, "found1");
        foundation2 = new Claw(hwMap, "found2");

        /*redLED1 = hwMap.get(DigitalChannel.class, "red1");
        greenLED1 = hwMap.get(DigitalChannel.class, "green1");
        redLED2 = hwMap.get(DigitalChannel.class, "red2");
        greenLED2 = hwMap.get(DigitalChannel.class, "green2");
        redLED1.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED1.setMode(DigitalChannel.Mode.OUTPUT);
        redLED2.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED2.setMode(DigitalChannel.Mode.OUTPUT);

        elbowMotor.init(RobotConstants.EL_EXT_MOT, -2);
        elbowMotor.setDir(RobotConstants.EL_DIR);
//        elbowMotor.setMode(STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setLevelOffset(
            RobotConstants.EL_LEVS[0],
            RobotConstants.EL_LEVS[1],
            RobotConstants.EL_LEVS[2],
            RobotConstants.EL_LEVS[3],
            RobotConstants.EL_LEVS[4],
            RobotConstants.EL_LEVS[5]);
        armLevel =0;

        extenderMotor.init(RobotConstants.EL_EX_MOT, 4.72);
        extenderMotor.setDir(RobotConstants.EXT_DIR);
        //extenderMotor.setMode(STOP_AND_RESET_ENCODER);
        extenderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extenderMotor.setLevelOffset(
            RobotConstants.EX_LEVS[0],
            RobotConstants.EX_LEVS[1]);

        sweeperServo1.init(RobotConstants.SWP_SRV);
        sweeperServo1.setDir(RobotConstants.SWP_DIR);

        sweeperServo2.init(RobotConstants.SWP_SRV);
        sweeperServo2.setDir(RobotConstants.SWP_DIR2);

        bucketServo.init(RobotConstants.SWP_SRV);
        bucketServo.setDir(RobotConstants.SWP_DIR2);
        bucketServo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        wristServo.init(0,.82);
        wristServo.moveAt(1);

        droneLauncherServo.init(0,.60);
        droneLauncherServo.moveAt(1.0);
        droneLauncherServo.moveTo(0.25);
        if (droneLauncherServo != null)
        {
            RobotLog.dd(TAG,"drone launcher was init");
        }
        intake.init();
        */
         intake.init();
         arm.init(RobotConstants.ARM_MOT,1);
        arm.setDir(RobotConstants.ARM_DIR);
        //arm.setMode(STOP_AND_RESET_ENCODER);        //TODO: make not happen when comming back from auton;
        arm.setLevelOffset(
                RobotConstants.ARM_LEVS[0],
                RobotConstants.ARM_LEVS[1],
                RobotConstants.ARM_LEVS[2],
                RobotConstants.ARM_LEVS[3],
                RobotConstants.ARM_LEVS[4],
                RobotConstants.ARM_LEVS[5],
                RobotConstants.ARM_LEVS[6]);


        armLevel =0;
        slides.init();
      //  slides.setMode(STOP_AND_RESET_ENCODER);        //TODO: make not happen when comming back from auton;
        //elbowMotor.init(RobotConstants.EL_EXT_M, -2);

    }

    private final ElapsedTime updTimer = new ElapsedTime();
    public void update()
    {
        updTimer.reset();
        super.update();
        double botTime = updTimer.milliseconds();
        updTimer.reset();
        if(intake != null)
        {
            intake.update();
        }
        if(slides != null)
        {
            slides.update();
        }
        if(arm != null)
        {
            arm.update();
        }
        double lftTime = updTimer.milliseconds();
        updTimer.reset();

        double clawTime;
//        if(claw != null)
//        {
//            claw.update();
//        }
        updTimer.reset();

//        if(VERBOSE) { RobotLog.dd(TAG, "UPD BOT:%.2f LFT:%.2f CLW:%.2f", botTime, lftTime, clawTime); }
        ledUpdate();
    }

    public void ledUpdate(){
        /*switch (pixelPieces){
            case 2:
                greenLED1.setState(false);
                redLED1.setState(false);
                greenLED2.setState(false);
                redLED2.setState(false);
                break;
            case 1:
            case 3:
                greenLED1.setState(false);
                redLED1.setState(true);
                greenLED2.setState(false);
                redLED2.setState(true);
                break;
            case 4:
                greenLED1.setState(true);
                redLED1.setState(false);
                greenLED2.setState(true);
                redLED2.setState(false);
                break;
            case 0:
                greenLED1.setState(true);
                redLED1.setState(true);
                greenLED2.setState(true);
                redLED2.setState(true);
                break;


        }*/
    }

   public void initArmMot() throws InterruptedException {

        arm.moveToCnt(2000,.02);

        Thread.sleep(1000);
        arm.setMode(STOP_AND_RESET_ENCODER);
        arm.setMode(RUN_USING_ENCODER);
        arm.moveToCnt(-300, 0.4);
        Thread.sleep(2000);
        arm.moveToCnt(-215, 0.1);
    }
    public void initIntake() {
        intake.setPwr(0);
        RobotLog.dd(TAG, "intake off init");
    }

    public void initSlides() throws InterruptedException {
       slides.initPos();
    }



    boolean extenderStopped = true;

    public void setExtenderPower(double input){
//        if(input >= .1 && extenderMotor.getCurEnc() < EX_MAX || input <= -.1 && extenderMotor.getCurEnc() > EX_MIN){
//            extenderMotor.moveAtControlRate(input);
//            extenderStopped = false;
//        }else{
//            if(!extenderStopped) {
//                extenderMotor.moveAtControlRate(0);
//                extenderStopped = true;
//            }
//        }

    }

    private enum servoState{
        STOPPED,
        FORWARD,
        BACKWARD
    }

    public boolean purplePixelDrop = false;

    private servoState bucketServoOn;
    Timer timer = new Timer();

public  int armLevel;
public  int slideLevel;

    //TODO: combine arm and slide together into a single level concept
    public void armLevelUp(){
        if(armLevel != ARM_NUM_LEVS - 1){  //change "-2" based on amount of levs needed
            armLevel ++;

            arm.moveToLevel(armLevel, 1);
            slides.moveToLevel(armLevel);

        }
    }

    public void armLevelDown() {
        if(armLevel != 0){
            armLevel--;


            arm.moveToLevel(armLevel, -1);
            slides.moveToLevel(armLevel);
        }
    }


    public void hangPos(){
        arm.moveToCnt(-900, 1);
        slides.moveToLevel(7);
    }

    public void specimenPos(){
        arm.moveToLevel(3, 0.8);
        slides.moveToLevel(3);
        armLevel = 3;
    }

    public void drivePos(){
        arm.moveToLevel(1, 0.8);
        slides.moveToLevel(1);
        armLevel = 1;
    }
    public void pickUp(){
        arm.moveToLevel(0, 0.8);
        slides.moveToLevel(0);
        armLevel = 0;
    }
    public void releaseFound(){
       foundation1.openClaw(0);
    }
    public void clampFound(){
        foundation1.closeClaw(0);
    }

    public double getElSpd(int targetEncoder){
        if(targetEncoder > slides.getLiftPos()){
            return EL_SPD_DWN;
        }else {
            return EL_SPD;
        }
    }

    public enum ElbowPositions {
        SAMPLE_MODE,
        SPECIMEN_MODE
    }

public ElbowPositions currentMode=ElbowPositions.SAMPLE_MODE;





}
