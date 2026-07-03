package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
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

public class


Constants {
    // 機器數值調整 請見 http://youtube.com/watch?v=vihb2LPtSK0&t
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(18)
            .forwardZeroPowerAcceleration(-33.8028238641209)
            .lateralZeroPowerAcceleration(-78.01025375605516)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08,0,0.02,0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1.55,0,0.06,0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0.0,0.00005,0.6,0.001))
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("mRF")
            .rightRearMotorName("mRR")
            .leftRearMotorName("mLR")
            .leftFrontMotorName("mLF")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(78.26583838275099)
            .yVelocity(63.16334689883735)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-112/25.4)
            .strafePodX(-72/25.4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("Pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

}
