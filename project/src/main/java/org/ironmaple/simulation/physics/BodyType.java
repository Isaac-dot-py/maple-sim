package org.ironmaple.simulation.physics;

/**
 *
 *
 * <h2>Body type enumeration for Box2D.</h2>
 */
public enum BodyType {
    /** A static body does not move and has infinite mass. */
    STATIC(0),

    /** A kinematic body moves according to its velocity but is not affected by forces. */
    KINEMATIC(1),

    /** A dynamic body is fully simulated. */
    DYNAMIC(2);

    private final int value;

    BodyType(int value) {
        this.value = value;
    }

    /**
     *
     *
     * <h2>Get the native value for this body type.</h2>
     *
     * @return The native int value
     */
    public int getValue() {
        return value;
    }

    /**
     *
     *
     * <h2>Get a BodyType from its native value.</h2>
     *
     * @param value The native int value
     * @return The corresponding BodyType
     */
    public static BodyType fromValue(int value) {
        return switch (value) {
            case 0 -> STATIC;
            case 1 -> KINEMATIC;
            case 2 -> DYNAMIC;
            default -> throw new IllegalArgumentException("Unknown body type: " + value);
        };
    }
}
