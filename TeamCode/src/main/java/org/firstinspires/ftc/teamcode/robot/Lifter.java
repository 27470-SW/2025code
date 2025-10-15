package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_LEVS;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_MAX_DEVIATION;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_MIN_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_NEAR_ZERO;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EX_MIN;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.INIT_SLIDE_POWER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.SLIDECPI;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.SLIDE_POWER;


public class Lifter
{
    public Lifter(String cfgName,String cfgName2, HardwareMap map)
    {
        this.name  = cfgName;
        this.name2 = cfgName2;
        this.hwMap = map;
    }

    public boolean init() {
        boolean success = false;


        try {
            liftMotor2 = hwMap.get(DcMotorEx.class, name2);
            liftMotor2.setDirection(RobotConstants.SLIDE2_DIR);

            RobotLog.dd(TAG, "initiolized liftmoter2 " + name);   //TODO: add liftMotor2

        } catch (Exception ignored) {
        }

        try {
            liftMotor = hwMap.get(DcMotorEx.class, name);
            liftMotor.setDirection(RobotConstants.SLIDE1_DIR);
            //setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setLiftPwr(0.0);            //setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMode(RUN_USING_ENCODER);
            success = true;
        }   catch (Exception ignored)

        {
        }
        try
        {
            //made for one servo
            liftServo = hwMap.get(CRServo.class, name);
            liftServo.setDirection(RobotConstants.LF_ARM_ROT_DIR);
            setLiftPwr(0.0);
            RobotLog.dd(TAG, "Lifter.init Found liftServo " + name);
            success = true;
        }
        catch (Exception ignored)
        {
        }

        if(!success) RobotLog.ee(TAG, "ERROR lifter - no liftMotor or liftServo " + name + " or " + name2);

        return success;
    }
    public int getCurEnc(){
       return liftMotor.getCurrentPosition();
    }
    public void initPos()throws InterruptedException{
        if (liftMotor2 != null) {
            liftMotor2.setMode(RUN_TO_POSITION);
            liftMotor2.setPower(INIT_SLIDE_POWER);
            liftMotor2.setTargetPosition(-2500);
        }
        liftMotor.setMode(RUN_TO_POSITION);
        liftMotor.setPower(INIT_SLIDE_POWER);
        liftMotor.setTargetPosition(-2500);
        Thread.sleep(2000);
        if (liftMotor2 != null) {
            liftMotor2.setMode(STOP_AND_RESET_ENCODER);
            liftMotor2.setMode(RUN_TO_POSITION);
            liftMotor2.setPower(SLIDE_POWER);
            liftMotor2.setTargetPosition(EX_MIN);
            RobotLog.dd(TAG, "lm2e: %d, lm2p: %f", liftMotor2.getCurrentPosition(), liftMotor2.getPower());

        }
        liftMotor.setMode(STOP_AND_RESET_ENCODER);
        liftMotor.setMode(RUN_TO_POSITION);
        liftMotor.setPower(SLIDE_POWER);
        liftMotor.setTargetPosition(EX_MIN);

        RobotLog.dd(TAG, "lm1e: %d, lm1p: %f ",liftMotor.getCurrentPosition(), liftMotor.getPower() );

    }


    public int getLiftPos()

   {
       if (liftMotor != null){
           return liftMotor.getCurrentPosition();
       }
       else {return 0;}

   }


    private boolean slidesOff = false;
    //pos in counts
    public void setLiftPos(int pos)
    {
        //Sets the position when it needs to be held up
        if (pos - EL_MIN_ENCODER < EL_NEAR_ZERO) {
//
                    slidesOff = true;
               } else{
            slidesOff = false;
        }

        {
            tgtCnt = pos;
            if (liftMotor != null) liftMotor.setTargetPosition(pos);
            if (liftMotor2 != null) liftMotor2.setTargetPosition(pos);

            if (lastRunMode != RUN_TO_POSITION) {
                setMode(RUN_TO_POSITION);
            }


            setLiftPwr(SLIDE_POWER);
        }

    }

    public void moveToLevel(int level){

        setLiftPos((int)Math.round(EL_LEVS[level] * SLIDECPI));


    }

    public void setLiftSpd(double pwr, double softLimit)
    {

        if (Math.abs(pwr) < 0.05 && lastRunMode != RUN_TO_POSITION)
        {
            setLiftPos(lftCnts);
            RobotLog.ee(TAG, "setting Lit Pos to: %d",lftCnts);

        }
        else if (Math.abs(pwr) >= 0.05)
        {
            slidesOff = false;
            int lm2Pos = 0;
            if(liftMotor2 != null)
            {
                lm2Pos = liftMotor2.getCurrentPosition();
            }

            if (lastRunMode != RUN_USING_ENCODER)
            {
                setMode(RUN_USING_ENCODER);
            }

            //safetycheck
            //TODO: replace with endstop check
            double lftmin = RobotConstants.EL_MIN_ENCODER;
            double lftmax = Math.min( RobotConstants.EL_MAX_ENCODER, softLimit);
            if (lftCnts <= lftmin && pwr < 0.0 ||
                lftCnts >= lftmax  && pwr > 0.0) pwr = 0.0;

            {
                setLiftPwr(pwr * SLIDE_POWER);
            }
        }
    }

    public void setMode(DcMotor.RunMode mode)
    {
        if (liftMotor != null && mode != lastRunMode) {
            liftMotor.setMode(mode);
            lastRunMode = mode;
            if (liftMotor2 != null)
            {
                liftMotor2.setMode(mode);
            }
        }
    }

    public void setLiftPwr(double pwr)
    {
        if(cmdPwr == pwr) return;
        if (liftMotor != null) liftMotor.setPower(pwr);
        if (liftMotor2 != null) liftMotor2.setPower(pwr);
        if (liftServo != null) liftServo.setPower(pwr);
        cmdPwr = pwr;

        //Could use a pwr*maxAllowedDps and the LIFTER_CPD  to get CPS then use setVelocity
    }

    public void update()
    {
        if(liftMotor != null){

            lftCnts = liftMotor.getCurrentPosition();
            if(liftMotor2 != null) {
                int motor2cnts = liftMotor2.getCurrentPosition();
                if (lftCnts > motor2cnts + EL_MAX_DEVIATION || lftCnts < motor2cnts - EL_MAX_DEVIATION)
                    RobotLog.dd(TAG, "Warning slides not on same motor count:slide1 = " + lftCnts + " slide2 = " + liftMotor2.getCurrentPosition());

                if (lftCnts - EL_MIN_ENCODER < EL_NEAR_ZERO && slidesOff) {
                    RobotLog.dd(TAG, "Slides near bottom, disengaging. slide1 = " + lftCnts + " slide2 = " + motor2cnts);
                    setLiftPwr(0.0);
                    setMode(RUN_USING_ENCODER);
               }
            }

        }

    }


    public String toString()
    {
        return String.format(Locale.US,
                "lift %5d %5d %.2f %s",
                lftCnts, tgtCnt, cmdPwr, lastRunMode);
    }
    public DcMotor.RunMode getMode(){
       return liftMotor.getMode();

    }
    private int encoderPos;
    public void holdPosition(){
        encoderPos = liftMotor.getCurrentPosition();

        liftMotor.setMode(RUN_TO_POSITION);
        liftMotor.setTargetPosition(encoderPos);
        liftMotor.setPower(SLIDE_POWER);

        if (liftMotor2 != null) {
            liftMotor2.setMode(RUN_TO_POSITION);
            liftMotor2.setTargetPosition(encoderPos);
            liftMotor2.setPower(SLIDE_POWER);
        }
    }
    private DcMotorEx liftMotor;
    private DcMotorEx liftMotor2;
    private CRServo liftServo;
    protected HardwareMap hwMap;
    protected final String name;
    protected final String name2;

    private DcMotor.RunMode lastRunMode;
    private int tgtCnt = 0;
    private int lftCnts = 0;
//    private double tgtVel = 0.0;
    private double cmdPwr = Double.MAX_VALUE;

    private static final double LIFTER_CPER = 28; //quad encoder cnts/encoder rev
    private static final Motors.MotorModel mod = Motors.MotorModel.GOBILDA_5202_71_2;
    private static final double LIFTER_INT_GEAR = mod.getGear();
    private static final double LIFTER_EXT_GEAR = RobotConstants.LF_ARM_ROT_GEAR;
    private static final double LIFTER_CPR = LIFTER_CPER * LIFTER_INT_GEAR * LIFTER_EXT_GEAR; // cnts/outShaftRev
    private static final double LIFTER_CPD = LIFTER_CPR / 360.0;
    private static final int    LIFTER_OFST = (int)(RobotConstants.LF_STOW_DEG * LIFTER_CPD);

    private static final String TAG = "SJH_LFT";
}
