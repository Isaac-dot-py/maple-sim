package org.ironmaple.utils.mathutils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;

/** utils to convert between WPILIB and Box2D geometry classes */
public class GeometryConvertor {
    public static Vec2 toBox2dVec2(Translation2d wpilibTranslation2d) {
        return new Vec2((float) wpilibTranslation2d.getX(), (float) wpilibTranslation2d.getY());
    }

    public static Translation2d toWpilibTranslation2d(Vec2 box2dVec2) {
        return new Translation2d(box2dVec2.x, box2dVec2.y);
    }

    public static float toBox2dRotation(Rotation2d wpilibRotation2d) {
        return (float) wpilibRotation2d.getRadians();
    }

    public static Rotation2d toWpilibRotation2d(float box2dRotation) {
        return new Rotation2d(box2dRotation);
    }

    public static Pose2d toWpilibPose2d(Transform box2dTransform) {
        return new Pose2d(
                toWpilibTranslation2d(box2dTransform.p), toWpilibRotation2d(box2dTransform.q.getAngle()));
    }

    public static Vec2 toBox2dLinearVelocity(ChassisSpeeds wpilibChassisSpeeds) {
        return new Vec2((float) wpilibChassisSpeeds.vxMetersPerSecond, (float) wpilibChassisSpeeds.vyMetersPerSecond);
    }

    public static ChassisSpeeds toWpilibChassisSpeeds(Vec2 box2dLinearVelocity, double angularVelocityRadPerSec) {
        return new ChassisSpeeds(box2dLinearVelocity.x, box2dLinearVelocity.y, angularVelocityRadPerSec);
    }

    public static Translation2d getChassisSpeedsTranslationalComponent(ChassisSpeeds chassisSpeeds) {
        return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }
}
