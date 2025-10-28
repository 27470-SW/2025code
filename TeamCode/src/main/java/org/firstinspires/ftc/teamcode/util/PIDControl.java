package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDControl {

    private final HardwareMap hwMap;
    DcMotorEx motor = null;
    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double targetVelocity;

    public PIDControl(HardwareMap map,String motorName)
    {
        this.hwMap = map;
        motor = hwMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);



    }

    public double getVelocity(){
        return motor.getVelocity();
    }

    public void setWheelVelocity(double targetVelocity){
        this.targetVelocity= targetVelocity;


    }

   public void update (){
       double power = PIDControlVoltage(targetVelocity,motor.getVelocity());
       motor.setPower(power);

       timer.reset();
    }

    public boolean init (double Kp, double Ki, double Kd,double Kf){
        this.Kp=Kp;
        this.Kd=Kd;
        this.Ki=Ki;
        this.Kf=Kf;

        return true;
    }





    public double PIDControlVoltage(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }
}