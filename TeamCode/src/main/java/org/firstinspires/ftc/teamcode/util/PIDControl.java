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
    public double targetVelocity;
    Input_Shaper ishaper;

    public PIDControl(HardwareMap map,String motorName)
    {
        this.hwMap = map;
        motor = hwMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



    }

    public double getVelocity(){
        return motor.getVelocity();
    }

    public double setWheelVelocity(double targetVelocity){
        this.targetVelocity= targetVelocity;


        return targetVelocity;
    }

    double vel0 = 0;
    double vel1 = 0;
    double vel2 = 0;
    double vel3 = 0;


    public double filteredVelocity = 0;
    double a = 0.4;
    double b = 0.25;
    double c = 0.2;
    double d = 0.15;
   public double update () {

       vel3 = vel2;
       vel2 = vel1;
       vel1 = vel0;
       vel0 = motor.getVelocity();

       filteredVelocity = a * vel0 + b * vel1 + c * vel2 + d * vel3;
       double power = PIDControlVoltage(targetVelocity, filteredVelocity);
       power = ishaper.shape(power, 0.02);

       timer.reset();
       if (targetVelocity != 0) {
           motor.setPower(power);
           return power;
       } else {
           motor.setPower(0);
           return 0;
       }




    }

    public double getFilteredVelocity(){
       return filteredVelocity;
    }

    public boolean init (double Kp, double Ki, double Kd,double Kf){
        this.Kp=Kp;
        this.Kd=Kd;
        this.Ki=Ki;
        this.Kf=Kf;

        ishaper = new Input_Shaper();
        return true;
    }

    public double getIntegralSum (){
       return integralSum;
    }


    public double PIDControlVoltage(double reference, double state) {
        double error = reference - state;
        if(Math.signum(error) != Math.signum(lastError)) integralSum = 0;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }
}