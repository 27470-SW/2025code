package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

public class Shooter
{
    public Shooter(HardwareMap map)
    {
        this.hwMap = map;
    }

    public boolean init()
    {
        boolean success = false;
        try
        {
            shooter = hwMap.get(DcMotorEx.class, "shoot");
            shooter.setDirection(DcMotor.Direction.REVERSE);
            shooter.setPower(0);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(RUN_USING_ENCODER);
            shtmode = RUN_USING_ENCODER;
            success = true;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initShooter\n" + e.toString());
        }
        try
        {
            moveShooter1 = hwMap.get(Servo.class, "shooterTraj1");
            moveShooter1.setPosition(0);
            moveShooter2 = hwMap.get(Servo.class, "shooterTraj2");
            moveShooter2.setPosition(0);
            success = true;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initShooter\n" + e.toString());
        }
        try
        {
            moveShooterM = hwMap.get(DcMotorEx.class, "shooterTrajM");
            moveShooterM.setDirection(DcMotor.Direction.FORWARD);
            moveShooterM.setPower(0);
            moveShooterM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            moveShooterM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            moveShooterM.setMode(RUN_USING_ENCODER);
            success = true;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initShooter\n" + e.toString());

        }

        shooter1 = new Transition("shooter1", hwMap);
        shooter2 = new Transition("shooter2", hwMap);
        shooter3 = new Transition("shooter3", hwMap);
        if(shooter1 != null) shooter1.init();
        if(shooter2 != null) shooter2.init();
        if(shooter3 != null) shooter3.init();

        setPIDF(new PIDFCoefficients(80.0, 0.0, 0.0,14.9));

        RobotLog.dd(TAG, "RUN_USING_ENC shooter PID. %s", shtPid);

        return success;
    }

    public void update()
    {
        if(shooter != null)
        {
            encPos = shooter.getCurrentPosition();
            curSpd = shooter.getVelocity();
        }
        if(shooter1 != null) shooter1.update();
        if(shooter2 != null) shooter2.update();
        if(shooter3 != null) shooter3.update();
    }

    public String toString()
    {
        return String.format(Locale.US, "shoot %5d %4.2f %4.2f %4.2f",
                encPos, curSpd, cps, dist);
    }

    public void stop()
    {
        cps = 0.0;
        if(shooter != null) shooter.setVelocity(cps);
        if(moveShooter1 != null) moveShooter1.setPosition(moveShooter1.getPosition());
        if(moveShooter2 != null) moveShooter2.setPosition(moveShooter2.getPosition());
        if(moveShooterM != null) moveShooterM.setVelocity(cps);
        shooter1.stop();
        shooter2.stop();
        shooter3.stop();
    }
public void stopWheel(){
        cps = 0.0;
    if(shooter != null) shooter.setVelocity(cps);
}
    private static final double g = -9.81 *3.28084 *12;
    private static final double height = 35;
    private static final double heightOfShooter = 10;
    private static final double dia = 4; //diameter of the wheels
    private static final double cir = dia * Math.PI; //cicumference of the wheels
    private static final double theta = Math.toRadians(35);
    private double calcCps(double distance)
    {
        v0 = Math.sqrt((g*Math.pow(distance,2))/
            (2*Math.pow(Math.cos(theta),2)*(height-distance*Math.tan(theta)-heightOfShooter)));
        return 2 * (v0 / cir) * SHOOTER_CPR;
    }

    public void setDistance(double distance)
    {
        dist = distance;
        cps = 1;
        if(shooter != null) shooter.setVelocity(cps);
    }

    public void setShootMode(DcMotor.RunMode mode)
    {
        if(shtmode != mode && shooter != null) shooter.setMode(mode);
        shtmode = mode;
    }

    public void changeShootTraj(double speed){
        try {
            if (moveShooter1.getPosition() == 0 && speed < 0) return;
            if (moveShooter1.getPosition() == 1 && speed > 0) return;
            moveShooter1.setPosition(speed / 100 + moveShooter1.getPosition());
            moveShooter1.setPosition(speed / 100 + moveShooter1.getPosition());
            moveShooterM.setPower(speed);
        }catch(Exception e){
            RobotLog.dd(TAG, "unable to change Shoot Trajectory");
        }
    }

    public void shootPower(double pwr)
    {
        setShootMode(RUN_WITHOUT_ENCODER);
        if(shooter != null) shooter.setPower(pwr);
    }

    public void shootCps(double cps)
    {
        this.cps = cps;
        if(shooter != null) shooter.setVelocity(cps);
    }

    public void shoot(BALL_CHOICE ball){
        switch (ball){
            case LEFT:
                shooter1.startTransition();
                break;
            case CENTER:
                shooter2.startTransition();
                break;
            case RIGHT:
                shooter3.startTransition();
                break;
        }
    }

    public double getV0() {return v0;}
    public int getEncPos() {return encPos;}
    public double getCurSpd() {return curSpd;}
    public double getCmdSpd() {return cps;}
    public double getDist() {return dist;}

    @SuppressWarnings("unused")
    public PIDFCoefficients getPidf() {return shtPid;}
    public void setPIDF(PIDFCoefficients pidf)
    {
        shtPid = pidf;
        if(shooter == null) return;
        shooter.setPIDFCoefficients(RUN_USING_ENCODER, shtPid);
    }

    private final double SHOOTER_CPER = 28; //quad encoder cnts/encoder rev
    private final double SHOOTER_INT_GEAR = 1; //1:1 motor - approx 6000 rpm (no load)
    private final double SHOOTER_EXT_GEAR = 1.0;
    private final double SHOOTER_CPR = SHOOTER_CPER * SHOOTER_INT_GEAR * SHOOTER_EXT_GEAR;
    private int encPos = 0;
    private double curSpd = 0;
    protected HardwareMap hwMap;
    public DcMotorEx shooter = null;
    public DcMotorEx moveShooterM = null;
    private Servo moveShooter1 = null;
    private Servo moveShooter2 = null;
    private Transition shooter1 = null;
    private Transition shooter2 = null;
    private Transition shooter3 = null;
    private static final String TAG = "SJH_SHT";
    private double dist = 0;
    private double cps = 0;
    private double v0 = 0.0;

    private DcMotor.RunMode shtmode = RUN_USING_ENCODER;

    private PIDFCoefficients shtPid = RobotConstants.SH_PID;

    public enum BALL_CHOICE {
        RIGHT,
        LEFT,
        CENTER
    }


}