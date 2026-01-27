package org.ironmaple.simulation.physics;

/**
 *
 *
 * <h2>Body for Box2D.</h2>
 *
 * <p>A rigid body. These are created via World.createBody().
 */
public class Body {
    /** Native pointer to b2Body. */
    long nativePtr;

    /** The parent world. */
    World world;

    /** User data. */
    Object userData;

    /** The fixture list head. */
    Fixture fixtureList;

    /** The next body in the world's body list. */
    Body next;

    /**
     *
     *
     * <h2>Package-private constructor.</h2>
     *
     * @param nativePtr The native pointer
     * @param world The parent world
     */
    Body(long nativePtr, World world) {
        this.nativePtr = nativePtr;
        this.world = world;
    }

    /**
     *
     *
     * <h2>Create a fixture from a shape.</h2>
     *
     * @param shape The shape
     * @param density The density in kg/m^2
     * @return The created fixture
     */
    public Fixture createFixture(Shape shape, float density) {
        FixtureDef def = new FixtureDef();
        def.shape = shape;
        def.density = density;
        return createFixture(def);
    }

    /**
     *
     *
     * <h2>Create a fixture from a fixture definition.</h2>
     *
     * @param def The fixture definition
     * @return The created fixture
     */
    public Fixture createFixture(FixtureDef def) {
        long fixturePtr = nativeCreateFixture(nativePtr, def);
        Fixture fixture = new Fixture(fixturePtr, this);
        fixture.userData = def.userData;
        fixture.next = fixtureList;
        fixtureList = fixture;
        return fixture;
    }

    /**
     *
     *
     * <h2>Destroy a fixture.</h2>
     *
     * @param fixture The fixture to destroy
     */
    public void destroyFixture(Fixture fixture) {
        // Remove from linked list
        if (fixtureList == fixture) {
            fixtureList = fixture.next;
        } else {
            Fixture f = fixtureList;
            while (f != null && f.next != fixture) {
                f = f.next;
            }
            if (f != null) {
                f.next = fixture.next;
            }
        }
        nativeDestroyFixture(nativePtr, fixture.nativePtr);
    }

    /**
     *
     *
     * <h2>Set the position and angle.</h2>
     *
     * @param position The world position
     * @param angle The angle in radians
     */
    public void setTransform(Vec2 position, float angle) {
        nativeSetTransform(nativePtr, position.x, position.y, angle);
    }

    /**
     *
     *
     * <h2>Get the body's transform.</h2>
     *
     * @return The transform
     */
    public Transform getTransform() {
        float[] data = nativeGetTransform(nativePtr);
        Transform t = new Transform();
        t.p.x = data[0];
        t.p.y = data[1];
        t.c = data[2];
        t.s = data[3];
        return t;
    }

    /**
     *
     *
     * <h2>Get the body's position.</h2>
     *
     * @return The position
     */
    public Vec2 getPosition() {
        float[] data = nativeGetPosition(nativePtr);
        return new Vec2(data[0], data[1]);
    }

    /**
     *
     *
     * <h2>Get the body's angle.</h2>
     *
     * @return The angle in radians
     */
    public float getAngle() {
        return nativeGetAngle(nativePtr);
    }

    /**
     *
     *
     * <h2>Get the center of mass in world coordinates.</h2>
     *
     * @return The world center
     */
    public Vec2 getWorldCenter() {
        float[] data = nativeGetWorldCenter(nativePtr);
        return new Vec2(data[0], data[1]);
    }

    /**
     *
     *
     * <h2>Get the local center of mass.</h2>
     *
     * @return The local center
     */
    public Vec2 getLocalCenter() {
        float[] data = nativeGetLocalCenter(nativePtr);
        return new Vec2(data[0], data[1]);
    }

    /**
     *
     *
     * <h2>Set the linear velocity.</h2>
     *
     * @param v The velocity vector
     */
    public void setLinearVelocity(Vec2 v) {
        nativeSetLinearVelocity(nativePtr, v.x, v.y);
    }

    /**
     *
     *
     * <h2>Get the linear velocity.</h2>
     *
     * @return The velocity vector
     */
    public Vec2 getLinearVelocity() {
        float[] data = nativeGetLinearVelocity(nativePtr);
        return new Vec2(data[0], data[1]);
    }

    /**
     *
     *
     * <h2>Set the angular velocity.</h2>
     *
     * @param omega The angular velocity in radians/second
     */
    public void setAngularVelocity(float omega) {
        nativeSetAngularVelocity(nativePtr, omega);
    }

    /**
     *
     *
     * <h2>Get the angular velocity.</h2>
     *
     * @return The angular velocity in radians/second
     */
    public float getAngularVelocity() {
        return nativeGetAngularVelocity(nativePtr);
    }

    /**
     *
     *
     * <h2>Apply a force at a world point.</h2>
     *
     * @param force The force vector in Newtons
     * @param point The world position of the force
     * @param wake Whether to wake the body
     */
    public void applyForce(Vec2 force, Vec2 point, boolean wake) {
        nativeApplyForce(nativePtr, force.x, force.y, point.x, point.y, wake);
    }

    /**
     *
     *
     * <h2>Apply a force at the center of mass.</h2>
     *
     * @param force The force vector in Newtons
     * @param wake Whether to wake the body
     */
    public void applyForceToCenter(Vec2 force, boolean wake) {
        nativeApplyForceToCenter(nativePtr, force.x, force.y, wake);
    }

    /**
     *
     *
     * <h2>Apply a torque.</h2>
     *
     * @param torque The torque in N-m
     * @param wake Whether to wake the body
     */
    public void applyTorque(float torque, boolean wake) {
        nativeApplyTorque(nativePtr, torque, wake);
    }

    /**
     *
     *
     * <h2>Apply an impulse at a world point.</h2>
     *
     * @param impulse The impulse vector in N-s
     * @param point The world position of the impulse
     * @param wake Whether to wake the body
     */
    public void applyLinearImpulse(Vec2 impulse, Vec2 point, boolean wake) {
        nativeApplyLinearImpulse(nativePtr, impulse.x, impulse.y, point.x, point.y, wake);
    }

    /**
     *
     *
     * <h2>Apply an impulse at the center of mass.</h2>
     *
     * @param impulse The impulse vector in N-s
     * @param wake Whether to wake the body
     */
    public void applyLinearImpulseToCenter(Vec2 impulse, boolean wake) {
        nativeApplyLinearImpulseToCenter(nativePtr, impulse.x, impulse.y, wake);
    }

    /**
     *
     *
     * <h2>Apply an angular impulse.</h2>
     *
     * @param impulse The angular impulse in kg*m*m/s
     * @param wake Whether to wake the body
     */
    public void applyAngularImpulse(float impulse, boolean wake) {
        nativeApplyAngularImpulse(nativePtr, impulse, wake);
    }

    /**
     *
     *
     * <h2>Get the total mass of the body.</h2>
     *
     * @return The mass in kilograms
     */
    public float getMass() {
        return nativeGetMass(nativePtr);
    }

    /**
     *
     *
     * <h2>Get the rotational inertia about the center of mass.</h2>
     *
     * @return The inertia in kg-m^2
     */
    public float getInertia() {
        return nativeGetInertia(nativePtr);
    }

    /**
     *
     *
     * <h2>Get the body type.</h2>
     *
     * @return The body type
     */
    public BodyType getType() {
        return BodyType.fromValue(nativeGetType(nativePtr));
    }

    /**
     *
     *
     * <h2>Set the body type.</h2>
     *
     * @param type The body type
     */
    public void setType(BodyType type) {
        nativeSetType(nativePtr, type.getValue());
    }

    /**
     *
     *
     * <h2>Is this body a bullet?</h2>
     *
     * @return True if bullet
     */
    public boolean isBullet() {
        return nativeIsBullet(nativePtr);
    }

    /**
     *
     *
     * <h2>Set if this body is a bullet.</h2>
     *
     * @param bullet True to set as bullet
     */
    public void setBullet(boolean bullet) {
        nativeSetBullet(nativePtr, bullet);
    }

    /**
     *
     *
     * <h2>Is this body sleeping allowed?</h2>
     *
     * @return True if sleep allowed
     */
    public boolean isSleepingAllowed() {
        return nativeIsSleepingAllowed(nativePtr);
    }

    /**
     *
     *
     * <h2>Set if this body is allowed to sleep.</h2>
     *
     * @param allowed True to allow sleep
     */
    public void setSleepingAllowed(boolean allowed) {
        nativeSetSleepingAllowed(nativePtr, allowed);
    }

    /**
     *
     *
     * <h2>Is this body awake?</h2>
     *
     * @return True if awake
     */
    public boolean isAwake() {
        return nativeIsAwake(nativePtr);
    }

    /**
     *
     *
     * <h2>Set if this body is awake.</h2>
     *
     * @param awake True to wake the body
     */
    public void setAwake(boolean awake) {
        nativeSetAwake(nativePtr, awake);
    }

    /**
     *
     *
     * <h2>Is this body active?</h2>
     *
     * @return True if active
     */
    public boolean isActive() {
        return nativeIsActive(nativePtr);
    }

    /**
     *
     *
     * <h2>Set if this body is active.</h2>
     *
     * @param active True to activate
     */
    public void setActive(boolean active) {
        nativeSetActive(nativePtr, active);
    }

    /**
     *
     *
     * <h2>Is this body fixed rotation?</h2>
     *
     * @return True if fixed rotation
     */
    public boolean isFixedRotation() {
        return nativeIsFixedRotation(nativePtr);
    }

    /**
     *
     *
     * <h2>Set if this body has fixed rotation.</h2>
     *
     * @param fixedRotation True to fix rotation
     */
    public void setFixedRotation(boolean fixedRotation) {
        nativeSetFixedRotation(nativePtr, fixedRotation);
    }

    /**
     *
     *
     * <h2>Get the user data.</h2>
     *
     * @return The user data
     */
    public Object getUserData() {
        return userData;
    }

    /**
     *
     *
     * <h2>Set the user data.</h2>
     *
     * @param userData The user data
     */
    public void setUserData(Object userData) {
        this.userData = userData;
    }

    /**
     *
     *
     * <h2>Get the parent world.</h2>
     *
     * @return The world
     */
    public World getWorld() {
        return world;
    }

    /**
     *
     *
     * <h2>Get the fixture list.</h2>
     *
     * @return The first fixture
     */
    public Fixture getFixtureList() {
        return fixtureList;
    }

    /**
     *
     *
     * <h2>Get the next body in the world's body list.</h2>
     *
     * @return The next body
     */
    public Body getNext() {
        return next;
    }

    /**
     *
     *
     * <h2>Set the linear damping.</h2>
     *
     * @param damping The damping
     */
    public void setLinearDamping(float damping) {
        nativeSetLinearDamping(nativePtr, damping);
    }

    /**
     *
     *
     * <h2>Get the linear damping.</h2>
     *
     * @return The damping
     */
    public float getLinearDamping() {
        return nativeGetLinearDamping(nativePtr);
    }

    /**
     *
     *
     * <h2>Set the angular damping.</h2>
     *
     * @param damping The damping
     */
    public void setAngularDamping(float damping) {
        nativeSetAngularDamping(nativePtr, damping);
    }

    /**
     *
     *
     * <h2>Get the angular damping.</h2>
     *
     * @return The damping
     */
    public float getAngularDamping() {
        return nativeGetAngularDamping(nativePtr);
    }

    /**
     *
     *
     * <h2>Get the gravity scale.</h2>
     *
     * @return The gravity scale
     */
    public float getGravityScale() {
        return nativeGetGravityScale(nativePtr);
    }

    /**
     *
     *
     * <h2>Set the gravity scale.</h2>
     *
     * @param scale The gravity scale
     */
    public void setGravityScale(float scale) {
        nativeSetGravityScale(nativePtr, scale);
    }

    // Native methods
    private static native long nativeCreateFixture(long bodyPtr, FixtureDef def);

    private static native void nativeDestroyFixture(long bodyPtr, long fixturePtr);

    private static native void nativeSetTransform(long ptr, float x, float y, float angle);

    private static native float[] nativeGetTransform(long ptr);

    private static native float[] nativeGetPosition(long ptr);

    private static native float nativeGetAngle(long ptr);

    private static native float[] nativeGetWorldCenter(long ptr);

    private static native float[] nativeGetLocalCenter(long ptr);

    private static native void nativeSetLinearVelocity(long ptr, float vx, float vy);

    private static native float[] nativeGetLinearVelocity(long ptr);

    private static native void nativeSetAngularVelocity(long ptr, float omega);

    private static native float nativeGetAngularVelocity(long ptr);

    private static native void nativeApplyForce(long ptr, float fx, float fy, float px, float py, boolean wake);

    private static native void nativeApplyForceToCenter(long ptr, float fx, float fy, boolean wake);

    private static native void nativeApplyTorque(long ptr, float torque, boolean wake);

    private static native void nativeApplyLinearImpulse(
            long ptr, float ix, float iy, float px, float py, boolean wake);

    private static native void nativeApplyLinearImpulseToCenter(long ptr, float ix, float iy, boolean wake);

    private static native void nativeApplyAngularImpulse(long ptr, float impulse, boolean wake);

    private static native float nativeGetMass(long ptr);

    private static native float nativeGetInertia(long ptr);

    private static native int nativeGetType(long ptr);

    private static native void nativeSetType(long ptr, int type);

    private static native boolean nativeIsBullet(long ptr);

    private static native void nativeSetBullet(long ptr, boolean bullet);

    private static native boolean nativeIsSleepingAllowed(long ptr);

    private static native void nativeSetSleepingAllowed(long ptr, boolean allowed);

    private static native boolean nativeIsAwake(long ptr);

    private static native void nativeSetAwake(long ptr, boolean awake);

    private static native boolean nativeIsActive(long ptr);

    private static native void nativeSetActive(long ptr, boolean active);

    private static native boolean nativeIsFixedRotation(long ptr);

    private static native void nativeSetFixedRotation(long ptr, boolean fixed);

    private static native void nativeSetLinearDamping(long ptr, float damping);

    private static native float nativeGetLinearDamping(long ptr);

    private static native void nativeSetAngularDamping(long ptr, float damping);

    private static native float nativeGetAngularDamping(long ptr);

    private static native float nativeGetGravityScale(long ptr);

    private static native void nativeSetGravityScale(long ptr, float scale);
}
