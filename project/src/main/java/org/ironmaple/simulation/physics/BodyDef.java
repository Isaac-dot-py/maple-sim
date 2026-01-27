package org.ironmaple.simulation.physics;

/**
 *
 *
 * <h2>Body definition for Box2D.</h2>
 *
 * <p>Contains the parameters needed to create a body.
 */
public class BodyDef {
    /** The body type: static, kinematic, or dynamic. Default is static. */
    public BodyType type = BodyType.STATIC;

    /** The world position of the body. */
    public Vec2 position = new Vec2();

    /** The world angle of the body in radians. */
    public float angle = 0;

    /** The linear velocity of the body's origin in world coordinates. */
    public Vec2 linearVelocity = new Vec2();

    /** The angular velocity of the body. */
    public float angularVelocity = 0;

    /** Linear damping - reduces linear velocity. */
    public float linearDamping = 0;

    /** Angular damping - reduces angular velocity. */
    public float angularDamping = 0;

    /** Set this flag to false if this body should never fall asleep. */
    public boolean allowSleep = true;

    /** Is this body initially awake or sleeping? */
    public boolean awake = true;

    /** Should this body be prevented from rotating? */
    public boolean fixedRotation = false;

    /** Is this a fast moving body that should be prevented from tunneling? */
    public boolean bullet = false;

    /** Does this body start out active? */
    public boolean active = true;

    /** User data pointer (for storing custom data) */
    public Object userData = null;

    /** Scale the gravity applied to this body. */
    public float gravityScale = 1.0f;
}
