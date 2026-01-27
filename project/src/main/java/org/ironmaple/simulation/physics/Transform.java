package org.ironmaple.simulation.physics;

/**
 *
 *
 * <h2>Transform for Box2D.</h2>
 *
 * <p>Represents a position and rotation in 2D space.
 */
public class Transform {
    /** Position */
    public final Vec2 p;

    /** Rotation cosine */
    public float c;

    /** Rotation sine */
    public float s;

    /**
     *
     *
     * <h2>Creates an identity transform.</h2>
     */
    public Transform() {
        this.p = new Vec2();
        this.c = 1;
        this.s = 0;
    }

    /**
     *
     *
     * <h2>Creates a transform with the given position and angle.</h2>
     *
     * @param position The position
     * @param angle The rotation angle in radians
     */
    public Transform(Vec2 position, float angle) {
        this.p = new Vec2(position);
        this.c = (float) Math.cos(angle);
        this.s = (float) Math.sin(angle);
    }

    /**
     *
     *
     * <h2>Set this transform to the identity.</h2>
     */
    public void setIdentity() {
        p.setZero();
        c = 1;
        s = 0;
    }

    /**
     *
     *
     * <h2>Set the transform.</h2>
     *
     * @param position The position
     * @param angle The rotation angle in radians
     */
    public void set(Vec2 position, float angle) {
        p.set(position);
        c = (float) Math.cos(angle);
        s = (float) Math.sin(angle);
    }

    /**
     *
     *
     * <h2>Get the rotation angle.</h2>
     *
     * @return The angle in radians
     */
    public float getAngle() {
        return (float) Math.atan2(s, c);
    }

    /**
     *
     *
     * <h2>Transform a vector by this transform.</h2>
     *
     * @param v The vector to transform
     * @return The transformed vector
     */
    public Vec2 mul(Vec2 v) {
        return new Vec2(c * v.x - s * v.y + p.x, s * v.x + c * v.y + p.y);
    }

    /**
     *
     *
     * <h2>Inverse transform a vector by this transform.</h2>
     *
     * @param v The vector to inverse transform
     * @return The inverse transformed vector
     */
    public Vec2 mulTrans(Vec2 v) {
        float px = v.x - p.x;
        float py = v.y - p.y;
        return new Vec2(c * px + s * py, -s * px + c * py);
    }
}
