package org.ironmaple.simulation.physics;

/**
 *
 *
 * <h2>Contact manifold.</h2>
 *
 * <p>Contains contact point information.
 */
public class Manifold {
    /** The local normal. */
    public Vec2 localNormal = new Vec2();

    /** The local point. */
    public Vec2 localPoint = new Vec2();

    /** The manifold type. */
    public ManifoldType type = ManifoldType.CIRCLES;

    /** The number of points. */
    public int pointCount = 0;

    /**
     *
     *
     * <h2>Manifold type enumeration.</h2>
     */
    public enum ManifoldType {
        CIRCLES,
        FACE_A,
        FACE_B
    }
}
