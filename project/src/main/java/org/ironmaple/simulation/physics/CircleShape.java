package org.ironmaple.simulation.physics;

/**
 *
 *
 * <h2>Circle shape for Box2D.</h2>
 *
 * <p>A circle shape defined by a center point and radius.
 */
public class CircleShape extends Shape {
    /** The radius of the circle. */
    public float m_radius;

    /** The center position relative to the parent body. */
    public Vec2 m_p = new Vec2();

    /**
     *
     *
     * <h2>Creates a circle shape with zero radius.</h2>
     */
    public CircleShape() {
        m_radius = 0;
    }

    @Override
    public ShapeType getType() {
        return ShapeType.CIRCLE;
    }

    @Override
    public int getChildCount() {
        return 1;
    }

    @Override
    public CircleShape clone() {
        CircleShape shape = new CircleShape();
        shape.m_radius = this.m_radius;
        shape.m_p = new Vec2(this.m_p);
        return shape;
    }

    /**
     *
     *
     * <h2>Set the radius of the circle.</h2>
     *
     * @param radius The radius in meters
     */
    public void setRadius(float radius) {
        this.m_radius = radius;
    }

    /**
     *
     *
     * <h2>Get the radius of the circle.</h2>
     *
     * @return The radius in meters
     */
    public float getRadius() {
        return m_radius;
    }
}
