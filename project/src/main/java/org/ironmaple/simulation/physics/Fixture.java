package org.ironmaple.simulation.physics;

/**
 *
 *
 * <h2>Fixture for Box2D.</h2>
 *
 * <p>A fixture binds a shape to a body and adds material properties.
 */
public class Fixture {
    /** Native pointer to b2Fixture. */
    long nativePtr;

    /** The parent body. */
    Body body;

    /** User data. */
    Object userData;

    /** The next fixture in the linked list. */
    Fixture next;

    /**
     *
     *
     * <h2>Package-private constructor.</h2>
     *
     * @param nativePtr The native pointer
     * @param body The parent body
     */
    Fixture(long nativePtr, Body body) {
        this.nativePtr = nativePtr;
        this.body = body;
    }

    /**
     *
     *
     * <h2>Get the parent body.</h2>
     *
     * @return The parent body
     */
    public Body getBody() {
        return body;
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
     * <h2>Is this fixture a sensor?</h2>
     *
     * @return True if sensor
     */
    public boolean isSensor() {
        return nativeIsSensor(nativePtr);
    }

    /**
     *
     *
     * <h2>Set if this fixture is a sensor.</h2>
     *
     * @param sensor True to make it a sensor
     */
    public void setSensor(boolean sensor) {
        nativeSetSensor(nativePtr, sensor);
    }

    /**
     *
     *
     * <h2>Get the friction.</h2>
     *
     * @return The friction coefficient
     */
    public float getFriction() {
        return nativeGetFriction(nativePtr);
    }

    /**
     *
     *
     * <h2>Set the friction.</h2>
     *
     * @param friction The friction coefficient
     */
    public void setFriction(float friction) {
        nativeSetFriction(nativePtr, friction);
    }

    /**
     *
     *
     * <h2>Get the restitution.</h2>
     *
     * @return The restitution
     */
    public float getRestitution() {
        return nativeGetRestitution(nativePtr);
    }

    /**
     *
     *
     * <h2>Set the restitution.</h2>
     *
     * @param restitution The restitution
     */
    public void setRestitution(float restitution) {
        nativeSetRestitution(nativePtr, restitution);
    }

    /**
     *
     *
     * <h2>Get the density.</h2>
     *
     * @return The density in kg/m^2
     */
    public float getDensity() {
        return nativeGetDensity(nativePtr);
    }

    /**
     *
     *
     * <h2>Set the density.</h2>
     *
     * @param density The density in kg/m^2
     */
    public void setDensity(float density) {
        nativeSetDensity(nativePtr, density);
    }

    /**
     *
     *
     * <h2>Get the next fixture in the body's fixture list.</h2>
     *
     * @return The next fixture, or null
     */
    public Fixture getNext() {
        return next;
    }

    // Native methods
    private static native boolean nativeIsSensor(long ptr);

    private static native void nativeSetSensor(long ptr, boolean sensor);

    private static native float nativeGetFriction(long ptr);

    private static native void nativeSetFriction(long ptr, float friction);

    private static native float nativeGetRestitution(long ptr);

    private static native void nativeSetRestitution(long ptr, float restitution);

    private static native float nativeGetDensity(long ptr);

    private static native void nativeSetDensity(long ptr, float density);
}
