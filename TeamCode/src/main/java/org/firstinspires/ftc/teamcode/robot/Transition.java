package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.RobotConstants.TRANSITION_MOVEMENT;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.info;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.RobotLog;


public class Transition {

    public Transition(String servoName, HardwareMap map)
    {
        this.hwMap = map;
        try {
            transitionServo = hwMap.get(Servo.class, servoName);
        }catch(Exception e){
            RobotLog.ee(TAG, "Unable to find transition");
        }
        percent = 1;
    }

    public void setTransitionPos(double pos)
    {
        if(transitionServo == null) return;
        transitionServo.setPosition(pos);
        percent = pos;
    }

    public void moveToStartPos()
    {
        setTransitionPos(TRANSITION_STARTPOINT);
    }

    public void update(){
        try {
  //          double pos = transitionServo.getPosition();
  //          if (TRANSITION_ENDPOINT == transitionServo.getPosition()) {
  //              percent = TRANSITION_RESTPOINT;
  //              RobotLog.dd(TAG, "in transition Update: Percent:%f POS:%f", percent, pos);
  //          }
            setTransitionPos(percent);
        }catch(Exception e){}
    }

    public void startTransition(){
        RobotLog.dd(TAG, "startTransition Percent:%f pos:%f", percent, transitionServo.getPosition());
        init(TRANSITION_STARTPOINT, forward);
        percent = TRANSITION_ENDPOINT;

        setTransitionPos(percent);
    }

    public void moveTransitionLittle(double amount){
        percent = percent + amount;
        setTransitionPos(percent);
        RobotLog.dd(TAG, "moving to position: %f", percent);
    }

    public void stop(){
        percent = transitionServo.getPosition();
        setTransitionPos(percent);
    }

    public boolean init(double start, boolean forward){
        setTransitionPos(start);
        TRANSITION_STARTPOINT = start;
        this.forward = forward;
        if(forward) {
            TRANSITION_ENDPOINT = start + TRANSITION_MOVEMENT;
        }else {
            TRANSITION_ENDPOINT = start - TRANSITION_MOVEMENT;
        }
        RobotLog.dd(TAG, "init Transition Percent:%f pos:%f", percent, transitionServo.getPosition());
        return true;
    }

private boolean forward;
    public double percent;
    private static final String TAG = "Transition";
    public Servo transitionServo;
    protected HardwareMap hwMap;
    public double maxOpen;
    public double maxClosed;
    public boolean Triggr;
    public int K1R2;
    private double TRANSITION_ENDPOINT;
    private double TRANSITION_STARTPOINT;

}
