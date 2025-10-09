package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;


public class ServoIntake {

    public ServoIntake(HardwareMap map)
    {
        this.hwMap = map;
        try {
            servoIntakeOne = hwMap.get(Servo.class, "Intakeservo1");
            servoIntakeTwo = hwMap.get(Servo.class, "Intakeservo2");
        }catch(Exception e){
            RobotLog.ee(TAG, "Unable to find claw");
        }
        percent = 0.5;
        onIntake = 1;
        offIntake = 0.5;


    }


    public void setIntakePower(double pos)
    {
        if(servoIntakeOne == null) return;
        servoIntakeOne.setPosition(pos);
        RobotLog.dd(TAG, "Percent:%f POS:%f", percent, pos);
        pos = 1-pos;
        if(servoIntakeTwo == null) return;
        servoIntakeTwo.setPosition(pos);
        RobotLog.dd(TAG, "Percent:%f POS:%f", percent, pos);

    }





public void update(){
        setIntakePower(percent);
}

    public void intakeOn(double val){
        RobotLog.dd(TAG, "Percent:%f Val:%f", percent, val);

        percent = onIntake*val;

        setIntakePower(percent);
    }


    public void intakeOff(){

        percent = offIntake;

        setIntakePower(percent);
    }





    public double percent;
    private static final String TAG = "servoIntake";
    public Servo servoIntakeTwo;
    public Servo servoIntakeOne;

    protected HardwareMap hwMap;
    public double onIntake;
    public double offIntake;
    public boolean Triggr;
    public int K1R2;
}
