package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.RobotConstants.TRANSITION_RESTPOINT;

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
        pos = 1-pos;
        if(transitionServo == null) return;
        transitionServo.setPosition(pos);
        RobotLog.dd(TAG, "Percent:%f POS:%f", percent, pos);

    }

    public void update(){
        if(TRANSITION_ENDPOINT == transitionServo.getPosition())
        {
            percent = TRANSITION_RESTPOINT;
        }
        setTransitionPos(percent);
    }

    public void startTransition(){
        RobotLog.dd(TAG, "startTransition Percent:%f", percent);

        percent = TRANSITION_ENDPOINT;

        setTransitionPos(percent);
    }

    public void stop(){
        percent = transitionServo.getPosition();
        setTransitionPos(percent);
    }

    public boolean init(){
        setTransitionPos(TRANSITION_RESTPOINT);
        return true;
    }
    public double percent;
    private static final String TAG = "Transition";
    public Servo transitionServo;
    protected HardwareMap hwMap;
    public double maxOpen;
    public double maxClosed;
    public boolean Triggr;
    public int K1R2;
    private double TRANSITION_ENDPOINT = 0.26 + TRANSITION_RESTPOINT;
}
