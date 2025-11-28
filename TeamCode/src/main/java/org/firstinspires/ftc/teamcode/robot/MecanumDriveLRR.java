package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.util.CommonUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.robot.RobotConstants.HEADING_PID;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.LATERAL_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.DT_TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.TRANSLATIONAL_PID;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.kA;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.kStatic;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.kV;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@SuppressWarnings({"FieldMayBeFinal", "unused"})
@Config
public class MecanumDriveLRR extends MecanumDrive
{
    private final static String TAG = "SJH_MDL";

    private static boolean imuDebugging = false;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static TrajectoryVelocityConstraint velConstraint =
      getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, DT_TRACK_WIDTH);
    private static TrajectoryAccelerationConstraint accelConstraint =
      getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private BasicBot bot;

    public IMU imu;

    private VoltageSensor batteryVoltageSensor;

    private double followerTimeout = 0.5;

    public  MecanumDriveLRR()
    {
        this((IMU) null);
    }

    public MecanumDriveLRR(IMU imu)
    {
        super(kV, kA, kStatic, DT_TRACK_WIDTH, DT_TRACK_WIDTH, LATERAL_MULTIPLIER);

        RobotLog.dd(TAG, "kv %6.4f kA %6.4f DT_TRACK_W: %6.4f LATERAL_MULT: %4.4f",
                kV, kA, DT_TRACK_WIDTH,LATERAL_MULTIPLIER);


        this.imu = imu;
        HardwareMap hardwareMap = CommonUtil.getInstance().getHardwareMap();

        init(hardwareMap);
    }

    public MecanumDriveLRR(BasicBot bot)
    {
        super(kV, kA, kStatic, DT_TRACK_WIDTH, DT_TRACK_WIDTH, LATERAL_MULTIPLIER);
        HardwareMap hardwareMap = CommonUtil.getInstance().getHardwareMap();

        init(hardwareMap);
    }

    private void preInit()
    {
    }

    private void init(HardwareMap hardwareMap)
    {
        //LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
        velConstraint =
          getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, DT_TRACK_WIDTH);
        accelConstraint =
          getAccelerationConstraint(MAX_ACCEL);
        RobotLog.dd(TAG, "Init MAX_VEL %6.2f MAX_ACCEL %6.2f DT_TRACK_W: %4.2f",
                    MAX_VEL, MAX_ACCEL, DT_TRACK_WIDTH);

        RobotLog.dd(TAG, "Initialiazing MecanumDriveLRR.");

        follower =
            new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(1.2)), followerTimeout);

        RobotLog.dd(TAG, "TRANSLATIONAL PID kP %6.4f kI %6.4f kD %6.4f HEADING_PID kP %6.4f kI %6.4f kD %6.4f",
                TRANSLATIONAL_PID.kP, TRANSLATIONAL_PID.kI, TRANSLATIONAL_PID.kD,  HEADING_PID.kP, HEADING_PID.kI, HEADING_PID.kD);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        if (bot == null && imu == null)
        {
            if (imuDebugging) {
                RobotLog.dd(TAG, "Initializing IMU on the Control HUB");
            }
            imu = hardwareMap.get(IMU.class, "imu-new");
                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
            imu.initialize(parameters);
        }
        else
        {
            if (imuDebugging) {
                RobotLog.dd(TAG, "IMU NOT INITIALIZED IN Mecanum_LRR");
            }
        }

        try
        {
            leftFront = hardwareMap.get(DcMotorEx.class, "FL");
            leftRear = hardwareMap.get(DcMotorEx.class, "BL");
            rightRear = hardwareMap.get(DcMotorEx.class, "BR");
            rightFront = hardwareMap.get(DcMotorEx.class, "FR");

            leftFront.setDirection(RobotConstants.DT_FLDIR);
            leftRear.setDirection(RobotConstants.DT_BLDIR);
            rightRear.setDirection(RobotConstants.DT_BRDIR);
            rightFront.setDirection(RobotConstants.DT_FRDIR);
        }
        catch(Exception e)
        {
            RobotLog.ee(TAG, "DUDE - UNCOOL -MOTOR NOT FOUND");
        }

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors)
        {
            if (motor == null) continue;
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            RobotLog.dd(TAG, "Motor Direction: " + motor.getDirection());
        }

        setMotorPowers(0.0, 0.0, 0.0, 0.0);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RobotLog.dd(TAG, "RUN Using Encoders = %s", RUN_USING_ENCODER?"TRUE":"FALSE");

        if (RUN_USING_ENCODER)
        {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else
        {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }
        

        // SBHTODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        try {
            setLocalizer(new PinpointLocalizer(hardwareMap));
        }
        catch(Exception e)
        {
            RobotLog.ww(TAG, "No Pinpoint Localizer");
        }
        trajectorySequenceRunner =
            new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, velConstraint, accelConstraint);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
            startPose,
            velConstraint, accelConstraint,
            MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();
        while((angle - heading) < -Math.PI) angle += 2*Math.PI;
        while((angle - heading) >  Math.PI) angle -= 2*Math.PI;
        trajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(getPoseEstimate())
                .turn(angle)
                .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(trajectory.start())
                .addTrajectory(trajectory)
                .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence)
    {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            if (motor != null) motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            if (motor != null) motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }


    public void setRealPoseEstimate(Pose2d in){
        getLocalizer().setPoseEstimate(in);
    }

    public Pose2d getRealPoseEstimate(){
        return getLocalizer().getPoseEstimate();
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    public List<DcMotorEx> getMotors() {return motors;}

    //@NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        if (leftFront  != null) leftFront.setPower(v);
        if (leftRear   != null) leftRear.setPower(v1);
        if (leftRear   != null) rightRear.setPower(v2);
        if (rightFront != null) rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading()
    {
        if (imuDebugging) {
            RobotLog.dd(TAG, "getRawExternalHeading");
        }
        double rawExternHeading = 0.0;
		Orientation orientation;
        YawPitchRollAngles orientationIMU = imu.getRobotYawPitchRollAngles();

        if(bot == null && imu != null)
        {
			rawExternHeading = orientationIMU.getYaw(AngleUnit.RADIANS);
            if (imuDebugging) {
                RobotLog.dd(TAG, "Robot Orientation %6.4f", rawExternHeading);
            }
        }
        else if (bot != null)
        {
            if (imuDebugging) {
                RobotLog.dd(TAG, "bot != getGyroAngles");
            }
            orientation = bot.getGyroAngles();
			rawExternHeading = (double) orientation.firstAngle;
        }
        else
        {
            if (imuDebugging) {
                RobotLog.dd(TAG, "Default to new Orientation");
            }
            orientation = new Orientation();
			rawExternHeading = (double) orientation.firstAngle;
        }	


        return rawExternHeading;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface
        if (imuDebugging) {
            RobotLog.dd(TAG, "getExternalHeadingVelocity");
        }
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
		AngularVelocity angVelYawPitchRoll;
        double angVel = 0.0;
        if(bot == null && imu != null)
        {
            angVel = angularVelocity.zRotationRate;
            if (imuDebugging) {
                RobotLog.dd(TAG, "Angular Heading Velocity %6.4f", angVel);
            }
        }
        else if(bot != null)
        {
            if (imuDebugging) {
                RobotLog.dd(TAG, "bot != getGyroAngles");
            }
            angVelYawPitchRoll = bot.getGyroVelocity();
			angVel = (double) angVelYawPitchRoll.zRotationRate;
        }
        else
        {
            if (imuDebugging) {
                RobotLog.dd(TAG, "Default to new Orientation");
            }
            angVelYawPitchRoll = new AngularVelocity();
		    angVel = (double) angVelYawPitchRoll.zRotationRate;
        }

        return angVel;
    }

    public static TrajectoryVelocityConstraint
      getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(maxAngularVel),
            new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public void setFollowerTimeout(double timeout)
    {
        if(followerTimeout != timeout)
        {
            followerTimeout = timeout;
            follower =
                new HolonomicPIDVAFollower(TRANSLATIONAL_PID,  TRANSLATIONAL_PID, HEADING_PID,
                    new Pose2d(0.5, 0.5, Math.toRadians(5.0)), followerTimeout);
        }
    }


    public void cancelFollowing() {
        if(trajectorySequenceRunner.isBusy()) trajectorySequenceRunner.cancelFollowing();
    }

    public void drawRoute() {
        if(trajectorySequenceRunner != null) trajectorySequenceRunner.drawRoute();
    }

    public void draw(Pose2d pose) {
        if(trajectorySequenceRunner != null) trajectorySequenceRunner.draw(pose);
    }

    public void draw() {
        draw(getPoseEstimate());
    }

}
