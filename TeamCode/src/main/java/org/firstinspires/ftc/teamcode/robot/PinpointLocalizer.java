package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

public class PinpointLocalizer implements Localizer {
    private static final String TAG = "pinpoint";
    private final GoBildaPinpointDriver pinpoint;
    private Pose2d poseEstimate;
    private Pose2d poseVelocity;

    public PinpointLocalizer(HardwareMap hardwareMap) {
        // Initialize the Pinpoint driver
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // TODO: Set your odometry pod offsets here (in mm)
        // These are the distances from the robot's center to each odometry pod
        pinpoint.setOffsets(7, 1, DistanceUnit.INCH); // Example: X offset = -84mm, Y offset = -168mm

        // TODO: Set which pod is which
        // STANDARD: X pod is parallel to forward, Y pod is perpendicular (strafe)
        // REVERSED: Swap if your wiring is different
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // TODO: Reverse encoder directions if needed
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // Reset position to start
        pinpoint.resetPosAndIMU();

        poseEstimate = new Pose2d(0, 0, 0);
        poseVelocity = new Pose2d(0, 0, 0);
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        // Convert Road Runner Pose2d to Pinpoint format and set it
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                pose2d.getX(),
                pose2d.getY(),
                AngleUnit.RADIANS,
                pose2d.getHeading()
        ));
        poseEstimate = pose2d;
    }

    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    @Override
    public void update() {
        // Tell Pinpoint to update its data
        pinpoint.update();

        // Get the position from Pinpoint (in mm) and convert to inches for Road Runner
        Pose2D pinpointPose = pinpoint.getPosition();
        double x = pinpointPose.getX(DistanceUnit.INCH);
        double y = pinpointPose.getY(DistanceUnit.INCH);
        double heading = pinpointPose.getHeading(AngleUnit.RADIANS);

        poseEstimate = new Pose2d(x, y, heading);


        // Get velocity from Pinpoint
        double vx = pinpoint.getVelX(DistanceUnit.INCH);
        double vy = pinpoint.getVelY(DistanceUnit.INCH);
        double headingVel = pinpoint.getHeadingVelocity();

        poseVelocity = new Pose2d(vx, vy, headingVel);

    }
}