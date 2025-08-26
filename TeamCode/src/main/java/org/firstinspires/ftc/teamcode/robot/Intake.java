package org.firstinspires.ftc.teamcode.robot;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

public class Intake
{
    public Intake(HardwareMap map)
    {
        this.hwMap = map;
    }

    public boolean init()
    {
        boolean success = false;
        try
        {
            intaker = hwMap.get(DcMotorEx.class, "intake");
            intaker.setDirection(RobotConstants.IN_DIR);
            intaker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intaker.setPower(0);
            lastIntakePwr = 0.0;
            success = true;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map init Motor Intake\n" + e.toString());
        }

        try
        {
            intakeSrvo = hwMap.get(Servo.class, "sweeper1");
            RobotLog.dd(TAG, "malachi wants to see if this will do something");
           // intakeSrvo.init(RobotConstants.SWP_SRV);
            intakeSrvo.setDirection(FORWARD);
            intakeSrvo.setPosition(1.0);
            lastIntakePwr = 0.0;
            success = true;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map init Servo Intake 1\n" + e.toString());
        }
        try
        {
            intakeSrvo2 = hwMap.get(Servo.class, "sweeper2");
            RobotLog.dd(TAG, "malachi wants to see if this will do something");
            // intakeSrvo.init(RobotConstants.SWP_SRV);
            intakeSrvo2.setDirection(FORWARD);
            intakeSrvo2.setPosition(1.0);
            lastIntakePwr = 0.0;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map init Servo Intake 2\n" + e.toString());
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
        if(intaker != null)
        {
            encPos = intaker.getCurrentPosition();
            curSpd = intaker.getVelocity();
            curPwr = intaker.getPower();
        }
        else if(intakeSrvo != null)
        {

            curPwr = intakeSrvo.getPosition();
        }
    }
    public void stop()
    {
        setPwr(0.0);
    }

    public void setPwr(double pwr)
    {
        RobotLog.dd(TAG,"InsetPwr");
        if(intaker != null && pwr != lastIntakePwr)
        {
            intaker.setPower(pwr);
            lastIntakePwr = pwr;
        }
        else if(intakeSrvo != null && pwr != lastIntakePwr)
        {
            intakeSrvo.setPosition(pwr);
            intakeSrvo2.setPosition(pwr);
            lastIntakePwr = pwr;
            RobotLog.dd(TAG,"Intake1pwr: %f, Intake2pwr: %f, LastIntakepwr: %f", intakeSrvo.getPosition(),intakeSrvo2.getPosition(),lastIntakePwr);
        }
    }

    private DcMotorEx intaker;
    private Servo intakeSrvo;
    private Servo intakeSrvo2;

    protected HardwareMap hwMap;
    private static final String TAG = "SJH_INT";
    private int encPos = 0;
    private double curSpd = 0.0;
    private double curPwr = 0.0;
    private double lastIntakePwr = 0.0;
}
