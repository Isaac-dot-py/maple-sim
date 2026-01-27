package org.ironmaple.simulation.physics;

/**
 *
 *
 * <h2>Base shape class for Box2D.</h2>
 *
 * <p>All shapes derive from this class.
 */
public abstract class Shape {
    /** The native pointer to the b2Shape */
    protected long nativePtr;

    /**
     *
     *
     * <h2>Get the shape type.</h2>
     *
     * @return The shape type
     */
    public abstract ShapeType getType();

    /**
     *
     *
     * <h2>Get the number of child primitives.</h2>
     *
     * @return The child count
     */
    public abstract int getChildCount();

    /**
     *
     *
     * <h2>Clone the shape.</h2>
     *
     * @return A copy of this shape
     */
    @Override
    public abstract Shape clone();

    /**
     *
     *
     * <h2>Shape type enumeration.</h2>
     */
    public enum ShapeType {
        CIRCLE,
        EDGE,
        POLYGON,
        CHAIN
    }
}
