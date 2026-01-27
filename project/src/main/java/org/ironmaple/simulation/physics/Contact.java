package org.ironmaple.simulation.physics;

/**
 *
 *
 * <h2>Contact between two fixtures.</h2>
 */
public class Contact {
    /** Native pointer to b2Contact. */
    long nativePtr;

    /** Fixture A. */
    Fixture fixtureA;

    /** Fixture B. */
    Fixture fixtureB;

    /**
     *
     *
     * <h2>Package-private constructor.</h2>
     *
     * @param nativePtr The native pointer
     */
    Contact(long nativePtr) {
        this.nativePtr = nativePtr;
    }

    /**
     *
     *
     * <h2>Get fixture A.</h2>
     *
     * @return Fixture A
     */
    public Fixture getFixtureA() {
        return fixtureA;
    }

    /**
     *
     *
     * <h2>Get fixture B.</h2>
     *
     * @return Fixture B
     */
    public Fixture getFixtureB() {
        return fixtureB;
    }

    /**
     *
     *
     * <h2>Is this contact touching?</h2>
     *
     * @return True if touching
     */
    public boolean isTouching() {
        return nativeIsTouching(nativePtr);
    }

    /**
     *
     *
     * <h2>Is this contact enabled?</h2>
     *
     * @return True if enabled
     */
    public boolean isEnabled() {
        return nativeIsEnabled(nativePtr);
    }

    /**
     *
     *
     * <h2>Enable/disable this contact.</h2>
     *
     * @param enabled True to enable
     */
    public void setEnabled(boolean enabled) {
        nativeSetEnabled(nativePtr, enabled);
    }

    /**
     *
     *
     * <h2>Get the child index for fixture A.</h2>
     *
     * @return The child index
     */
    public int getChildIndexA() {
        return nativeGetChildIndexA(nativePtr);
    }

    /**
     *
     *
     * <h2>Get the child index for fixture B.</h2>
     *
     * @return The child index
     */
    public int getChildIndexB() {
        return nativeGetChildIndexB(nativePtr);
    }

    // Native methods
    private static native boolean nativeIsTouching(long ptr);

    private static native boolean nativeIsEnabled(long ptr);

    private static native void nativeSetEnabled(long ptr, boolean enabled);

    private static native int nativeGetChildIndexA(long ptr);

    private static native int nativeGetChildIndexB(long ptr);
}
