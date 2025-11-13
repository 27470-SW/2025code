package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.MAX_SHOOTER_DIST;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.MAX_TRAJ_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.MAX_W_SPEED;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.MIN_SHOOTER_DIST;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.MIN_TRAJ_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.MIN_W_SPEED;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.SHOOTER_KD;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.SHOOTER_KF;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.SHOOTER_KI;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.SHOOTER_KP;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.TRANSITION_RESTPOINT1;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.TRANSITION_RESTPOINT2;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.TRANSITION_RESTPOINT3;
import static org.firstinspires.ftc.teamcode.test.TestPIDshooter.targetVelocity;

import static java.lang.Math.signum;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDControl;

public class Shooter
{
    public Shooter(HardwareMap map)
    {
        this.hwMap = map;
    }

    public void initPos() throws InterruptedException
    {
        if (moveShooterM != null) {
//            moveShooterM.setMode(RUN_USING_ENCODER);
//            moveShooterM.setPower(.02);
//            Thread.sleep(2000);
            moveShooterM.setPower(0);
            moveShooterM.setMode(STOP_AND_RESET_ENCODER);
            moveShooterM.setMode(RUN_USING_ENCODER);
            shtmode = RUN_USING_ENCODER;
        }
    }
    public boolean initShooter()
    {
        boolean success = false;
        try
        {
            shooterW = hwMap.get(DcMotorEx.class, "shoot");
            shooterW.setDirection(DcMotor.Direction.REVERSE);
            shooterW.setPower(0);
            shooterW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooterW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterW.setMode(RUN_USING_ENCODER);
            shooterW2 = hwMap.get(DcMotorEx.class, "shoot2");
            shooterW2.setDirection(DcMotor.Direction.REVERSE);
            shooterW2.setPower(0);
            shooterW2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooterW2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterW2.setMode(RUN_USING_ENCODER);
            controlShooterW = new PIDControl(hwMap, "shoot");
            controlShooterW.init(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);

            success = true;
        }


        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initShooter\n" + e.toString());
        }
        try
        {
            //moveShooter1 = hwMap.get(Servo.class, "shooterTraj1");
            //moveShooter1.setPosition(0);
            //moveShooter2 = hwMap.get(Servo.class, "shooterTraj2");
            //moveShooter2.setPosition(0);
            success = true;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initShooter\n" + e.toString());
        }
        try
        {
            moveShooterM = hwMap.get(DcMotorEx.class, "shooterTrajM");
            moveShooterM.setDirection(DcMotor.Direction.REVERSE);
            moveShooterM.setPower(0);
            moveShooterM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            moveShooterM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            moveShooterM.setMode(RUN_USING_ENCODER);
            shtmode = RUN_USING_ENCODER;
            vs = hwMap.get(VoltageSensor.class, "Control Hub");
            success = true;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initShooter\n" + e.toString());

        }

        shooter1 = new Transition("shooter1", hwMap);
        shooter2 = new Transition("shooter2", hwMap);
        shooter3 = new Transition("shooter3", hwMap);
        if(shooter1 != null) shooter1.init(TRANSITION_RESTPOINT1, true);
        if(shooter2 != null) shooter2.init(TRANSITION_RESTPOINT2, true);
        if(shooter3 != null) shooter3.init(TRANSITION_RESTPOINT3, false);

        setPIDF(new PIDFCoefficients(80.0, 0.0, 0.0,14.9));

        RobotLog.dd(TAG, "RUN_USING_ENC shooter PID. %s", shtPid);

        return success;
    }

    public void update()
    {
        if(shooterW != null)
        {
            encPos = shooterW.getCurrentPosition();
            curSpd = shooterW.getVelocity();
        }
        if(shooterW2 != null)
        {
            encPos = shooterW2.getCurrentPosition();
            curSpd = shooterW2.getVelocity();
        }
        if(usePIDs && controlShooterW != null){
            if(dashTelemetry == null){
                dashboardShooter = FtcDashboard.getInstance();
                dashTelemetry = dashboardShooter.getTelemetry();
            }

           double update = controlShooterW.update();
            shooterW2.setPower(update);

            double currentVelocity = controlShooterW.getVelocity();
            double error = targetVelocity - currentVelocity;

            dashTelemetry.addData("targetVelocity", targetVelocity);
            dashTelemetry.addData("currentVelocity", currentVelocity);
            dashTelemetry.addData("error", error);
            dashTelemetry.addData("Power", update);
            dashTelemetry.addData("filteredVelocity", controlShooterW.getFilteredVelocity());
            dashTelemetry.update();
        }
        if (engageAutoTraj){
            setShooterTrajPos(getTrajEncoderDist());
        }
        if(shooter1 != null) shooter1.update();
        if(shooter2 != null) shooter2.update();
        if(shooter3 != null) shooter3.update();



    }
    public void onAutoTraj(){
        engageAutoTraj = true;
        moveShooterM.setTargetPosition(getTrajEncoderDist());
        moveShooterM.setMode(RUN_TO_POSITION);
        shtmode = RUN_TO_POSITION;
    }
    private int getTrajEncoderDist(){
        int value = (int) (MIN_TRAJ_ENCODER + dist * (MAX_TRAJ_ENCODER - MIN_TRAJ_ENCODER) / (MAX_SHOOTER_DIST - MIN_SHOOTER_DIST));
        RobotLog.dd(TAG, "Traj encoder dist = %d", value);
        return value;
    }

    public String toString()
    {
        return String.format(Locale.US, "shoot %5d %4.2f %4.2f %4.2f",
                encPos, curSpd, cps, dist);
    }

    public void stop()
    {
        cps = 0.0;
        if(shooterW != null) shooterW.setVelocity(cps);
        if(shooterW2 != null) shooterW2.setVelocity(cps);

        //if(moveShooter1 != null) moveShooter1.setPosition(moveShooter1.getPosition());
        //if(moveShooter2 != null) moveShooter2.setPosition(moveShooter2.getPosition());
        if(moveShooterM != null) moveShooterM.setVelocity(cps);
        shooter1.stop();
        shooter2.stop();
        shooter3.stop();
    }
public void stopWheel(){
        cps = 0.0;
    if(shooterW != null) shooterW.setVelocity(cps);
    if(shooterW2 != null) shooterW2.setVelocity(cps);

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

    boolean usePIDs = true;
    boolean useDistance = true;
    public double distanceWVelocity( double distance){
        return MIN_W_SPEED + distance * (MAX_W_SPEED - MIN_W_SPEED) / (MAX_SHOOTER_DIST - MIN_SHOOTER_DIST);
    }

    public void spinShooterAutonFar(){
        controlShooterW.setWheelVelocity(1550);
    }

    public void spinShooterW(double distance)
    {
        dist = distance;
        double voltage = vs.getVoltage();
        if(usePIDs) {
            if (controlShooterW != null) {
                if (useDistance) {
                    double distanceShooterVelocity = distanceWVelocity(distance);
                    controlShooterW.setWheelVelocity(distanceShooterVelocity);
                }
                else {
                    controlShooterW.setWheelVelocity(SHOOTER_VELOCITY);
                }
            }
        }
        else {
            if (shooterW != null) shooterW.setVelocity(SHOOTER_VELOCITY);
            if (shooterW2 != null) shooterW2.setVelocity(SHOOTER_VELOCITY);
        }
    }
    public double getFilteredVelocity(){

        return controlShooterW.filteredVelocity;
    }
    public void setShootMode(DcMotor.RunMode mode)
    {
        if(shooterW != null) shooterW.setMode(mode);
        if(shooterW2 != null) shooterW2.setMode(mode);
    }
    public void resetTransition() {
        shooter1.moveToStartPos();
        shooter2.moveToStartPos();
        shooter3.moveToStartPos();
        RobotLog.dd(TAG,"reseting transtions");
    }

    public void defaultFarShooterTraj(){

        setShooterTrajPos(-1250);
    }
    public void defaultCloseShooterTraj(){
        setShooterTrajPos(-600);
    }

    //set to a specific encoder count
    public void setShooterTrajPos(int targetpos){
        RobotLog.dd(TAG, "targetPos = %d, currentPos = %d, mode = %s, power = %f", targetpos, moveShooterM.getCurrentPosition(), moveShooterM.getMode().toString(), moveShooterM.getPower());
        moveShooterM.setTargetPosition(targetpos);

        if(shtmode == RUN_USING_ENCODER){
            moveShooterM.setPower(0);
            moveShooterM.setMode(RUN_TO_POSITION);
            shtmode = RUN_TO_POSITION;
        }
        int currentpos = moveShooterM.getCurrentPosition();

        moveShooterM.setPower( signum (targetpos - currentpos)*.3);

        RobotLog.dd(TAG, "targetPos = %d, currentPos = %d, signum = %f", targetpos, currentpos, signum(targetpos-currentpos));
    }


    public void changeShootTraj(double pwr){
        try {
           // if (moveShooter1.getPosition() == 0 && speed < 0) return;
            //if (moveShooter1.getPosition() == 1 && speed > 0) return;
            //moveShooter1.setPosition(speed / 100 + moveShooter1.getPosition());
            //moveShooter1.setPosition(speed / 100 + moveShooter1.getPosition());
            double lftmin = RobotConstants.SHOOTER_MIN_ENCODER;
            double lftmax =  RobotConstants.SHOOTER_MAX_ENCODER;
            double trajCounts= moveShooterM.getCurrentPosition();
            if (trajCounts <= lftmin && pwr < 0.0 ||
                    trajCounts >= lftmax  && pwr > 0.0) pwr = 0.0;

            if( shtmode != RUN_USING_ENCODER) {
                moveShooterM.setMode(RUN_USING_ENCODER);
                shtmode = RUN_USING_ENCODER;
            }

            moveShooterM.setPower(pwr);
            engageAutoTraj = false;

            RobotLog.dd(TAG, "Set power to speed :%f",pwr);

        }catch(Exception e){
            RobotLog.dd(TAG, "unable to change Shoot Trajectory");
        }
    }

    public void shootPower(double pwr)
    {
        setShootMode(RUN_WITHOUT_ENCODER);
        if(shooterW != null) shooterW.setPower(pwr);
        if(shooterW2 != null) shooterW2.setPower(pwr);

    }

    public void shootCps(double cps)
    {
        this.cps = cps;
        if(shooterW != null) shooterW.setVelocity(cps);
        if(shooterW2 != null) shooterW2.setVelocity(cps);

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
        if(shooterW == null) return;
        shooterW.setPIDFCoefficients(RUN_USING_ENCODER, shtPid);
        if(shooterW2 == null) return;
        shooterW2.setPIDFCoefficients(RUN_USING_ENCODER, shtPid);
    }

    private final double SHOOTER_CPER = 28; //quad encoder cnts/encoder rev
    private final double SHOOTER_INT_GEAR = 1; //1:1 motor - approx 6000 rpm (no load)
    private final double SHOOTER_EXT_GEAR = 1.0;
    private final double SHOOTER_CPR = SHOOTER_CPER * SHOOTER_INT_GEAR * SHOOTER_EXT_GEAR;
    private int encPos = 0;
    private double curSpd = 0;
    protected HardwareMap hwMap;
    public DcMotorEx shooterW = null;
    public DcMotorEx shooterW2 = null;
    private PIDControl controlShooterW = null;

    public DcMotorEx moveShooterM = null;
    private VoltageSensor vs;
    private Servo moveShooter1 = null;
    private Servo moveShooter2 = null;
    public Transition shooter1 = null;
    public Transition shooter2 = null;
    public Transition shooter3 = null;
    private static final String TAG = "SJH_SHT";
    private double dist = 0;
    private double cps = 0;
    private double v0 = 0.0;
    public boolean engageAutoTraj = false;

    private DcMotor.RunMode shtmode = RUN_USING_ENCODER;

    private PIDFCoefficients shtPid = RobotConstants.SH_PID;
    private Telemetry dashTelemetry;
    private FtcDashboard dashboardShooter;
    public enum BALL_CHOICE {
        RIGHT,
        LEFT,
        CENTER
    }


}