package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/// @param P: Error is multiplied by P to fix it(how aggressively it gets fixed)
/// @param I: Small errors are added/subtracted by I to reduce error
/// @param D: When the control hub cuts off power to the motor(if the robot is close to the target, cut power), higher = more oscillation(or could be too high P)
/// @param F: Original Power to account for friction


/// Tuning steps: FPDI

/// e	Error = target - current
/// target:	Desired position or velocity
/// current:Measured position or velocity
/// kP	Proportional gain (reacts to current error)
/// kI	Integral gain (reacts to accumulated error)
/// kD	Derivative gain (reacts to error change rate)
/// kF	Feedforward gain (predicts required power)
/// dt	Time step between updates

/// P = kP * e
/// I = kI * sum(e * dt)
/// D = kD * ((e - lastE) / dt)
/// F = kF * target

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()

            // 1 TODO: 1 GET MASS OF BOT(IN KILOGRAMS) by finding weight(in pounds), then converting to kilograms

            //weight: 30.08 lbs(average)
            .mass(13.19954)

            //TODO: 5 Run ForwardZeroPowerAccelerationTuner after finding max x and y velocity
            .forwardZeroPowerAcceleration(-30.260691682448705)

            //TODO: 6 Run LateralZeroPowerAccelerationTuner to find side deceleration rate
            .lateralZeroPowerAcceleration(-67.06771722288579)

            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)

            //TRANSLATIONAL PIDF COEFFICIENTS

            //NOTE: D MIGHT BE A LITTLE OFF
            .translationalPIDFCoefficients(new PIDFCoefficients(0.225, 0, 0.027, 0.024))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.40, 0, 0.05, 0.015))
            //TODO: RETUNE HEADING PIDF COEFFICIENTS(not perfect, but acceptable)
            .headingPIDFCoefficients(new PIDFCoefficients(.6, 0, 0.25, 0.025));
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("right_front")
            .rightRearMotorName("right_back")
            .leftRearMotorName("left_back")
            .leftFrontMotorName("left_front")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)


            //4 TODO: 4 RUN Forward and LateralVelocityTuner and input end result(max directional velocity)
            .xVelocity(-34.43354803006264)
            .yVelocity(-60.49884031422858);



    public static PinpointConstants localizerConstants = new PinpointConstants()

            // 2 TODO:  2 MEASURE DISTANCE BETWEEN STRAFE POD AND FORWARD POD
            .forwardPodY(3.75)
            .strafePodX(8.25)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

            // 3 TODO: 3  FIND ENCODER DIRECTION BY RUNNING LOCALIZATION TEST(SEE IF ROBOT MOVEMENT CORRELATES TO X/Y VALUES)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);






}

