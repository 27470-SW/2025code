package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

public class CrAzYintake
{
    public CrAzYintake(HardwareMap map)
    {
        this.hwMap = map;
    }

    public boolean init() {
        boolean success = false;
        try {
            //intakeMotor1 = hwMap.get(DcMotorEx.class, "im1");
            //intakeMotor1.setDirection(RobotConstants.IN_DIR);
            //intakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //intakeMotor1.setPower(0);
            //lastIntakePwr = 0.0;
            success = true;
        } catch (Exception e) {
            RobotLog.ee(TAG, "ERROR get hardware map initIntake\n" + e.toString());
        }

        try {
            //intakeMotor2 = hwMap.get(DcMotorEx.class, "im2");
            //intakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
            //intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //intakeMotor2.setPower(0);
            //lastIntakePwr = 0.0;
            success = true;
        } catch (Exception e) {
            RobotLog.ee(TAG, "ERROR get hardware map initIntake\n" + e.toString());
        }




        try {
            intakeServo1 = hwMap.get(Servo.class, "is1");
            intakeServo1.setPosition(0.5);
            lastIntakePwr = 0.0;
            success = true;
        } catch (Exception e) {
            RobotLog.ee(TAG, "ERROR get hardware map initIntake servo 1\n" + e.toString());
        }

        try {
            intakeServo2 = hwMap.get(Servo.class, "is2");
            intakeServo2.setPosition(0.5);
            lastIntakePwr = 0.0;
            success = true;
        } catch (Exception e) {
            RobotLog.ee(TAG, "ERROR get hardware map initIntake servo 2\n" + e.toString());
        }

        try {
            intakeServo3 = hwMap.get(Servo.class, "is3");
            intakeServo3.setPosition(0.5);
            lastIntakePwr = 0.0;
            success = true;
        } catch (Exception e) {
            RobotLog.ee(TAG, "ERROR get hardware map initIntake servo 3\n" + e.toString());
        }
        try {
            intakeServo4 = hwMap.get(Servo.class, "is4");
            intakeServo4.setPosition(0.5);
            lastIntakePwr = 0.0;
            success = true;
        } catch (Exception e) {
            RobotLog.ee(TAG, "ERROR get hardware map initIntake servo 4\n" + e.toString());
        }
        return success;
    }

    public String toString(){
        return String.format(Locale.US,
                "intake %5d %4.2f %4.2f",
                encPos, curSpd, curPwr);
    }

    public void update()
    {
        if (intakeServo1 != null)
        {
            curPwr=intakeServo1.getPosition();
        }
        if (intakeServo2 != null)
        {
            curPwr=intakeServo2.getPosition();
        }
        if (intakeServo3 != null)
        {
            curPwr=intakeServo3.getPosition();
        }
        if (intakeServo4 != null)
        {
            curPwr=intakeServo4.getPosition();
        }
        //if(intakeMotor1 != null)
        {
           // encPos = intakeMotor1.getCurrentPosition();
            //curSpd = intakeMotor1.getVelocity();
            //curPwr = intakeMotor1.getPower();
        }
        //if(intakeMotor2 != null)
        {
            //encPos = intakeMotor2.getCurrentPosition();
            //curSpd = intakeMotor2.getVelocity();
            //curPwr = intakeMotor2.getPower();
        }
    }


    public void stop()
    {
        setPwr(0.0);
    }

    public void setPwr(double pwr)
    {
        RobotLog.dd(TAG,"intakePower = :%f,lastintakepwr= :%f",pwr,lastIntakePwr);

        if(pwr != lastIntakePwr)
        {
            lastIntakePwr = pwr;
            if(intakeServo1 != null){
                intakeServo1.setPosition(.5-pwr/2);
                RobotLog.dd(TAG,"Intake1on");
            }
            if(intakeServo2 != null){
                intakeServo2.setPosition(.5+pwr/2);
                RobotLog.dd(TAG,"Intake2on");
            }
            if(intakeServo3 != null) {
                intakeServo3.setPosition(.5+pwr/2);
                RobotLog.dd(TAG,"Intake3on");

        }
            if(intakeServo4 != null){
                intakeServo4.setPosition(.5-pwr/2);
                RobotLog.dd(TAG,"Intake4on");

            }
            //if(intakeMotor1 != null)intakeMotor1.setPower(pwr);
            //if(intakeMotor2 != null)intakeMotor2.setPower(pwr);

        }


    }


    private Servo intakeServo1;
   private Servo intakeServo2;
    private Servo intakeServo3;
    private Servo intakeServo4;
   private  DcMotorEx intakeMotor1;
   private DcMotorEx intakeMotor2;



    private DcMotorEx intaker;
    private CRServo intakeSrvo;

    protected HardwareMap hwMap;
    private static final String TAG = "SJH_INT";
    private int encPos = 0;
    private double curSpd = 0.0;
    private double curPwr = 0.0;
    private double lastIntakePwr = 0.0;
}
