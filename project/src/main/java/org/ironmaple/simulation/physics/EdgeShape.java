package org.ironmaple.simulation.physics;

/**
 *
 *
 * <h2>Edge shape for Box2D.</h2>
 *
 * <p>A line segment shape between two vertices.
 */
public class EdgeShape extends Shape {
    /** The first vertex. */
    public Vec2 m_vertex1 = new Vec2();

    /** The second vertex. */
    public Vec2 m_vertex2 = new Vec2();

    /** Optional ghost vertex for chain shapes. */
    public Vec2 m_vertex0 = new Vec2();

    /** Optional ghost vertex for chain shapes. */
    public Vec2 m_vertex3 = new Vec2();

    /** Has ghost vertex 0. */
    public boolean m_hasVertex0 = false;

    /** Has ghost vertex 3. */
    public boolean m_hasVertex3 = false;

    /** The edge radius. */
    public float m_radius = 0.01f;

    /**
     *
     *
     * <h2>Creates an empty edge shape.</h2>
     */
    public EdgeShape() {}

    @Override
    public ShapeType getType() {
        return ShapeType.EDGE;
    }

    @Override
    public int getChildCount() {
        return 1;
    }

    @Override
    public EdgeShape clone() {
        EdgeShape shape = new EdgeShape();
        shape.m_vertex1 = new Vec2(this.m_vertex1);
        shape.m_vertex2 = new Vec2(this.m_vertex2);
        shape.m_vertex0 = new Vec2(this.m_vertex0);
        shape.m_vertex3 = new Vec2(this.m_vertex3);
        shape.m_hasVertex0 = this.m_hasVertex0;
        shape.m_hasVertex3 = this.m_hasVertex3;
        shape.m_radius = this.m_radius;
        return shape;
    }

    /**
     *
     *
     * <h2>Set the edge vertices.</h2>
     *
     * @param v1 The first vertex
     * @param v2 The second vertex
     */
    public void set(Vec2 v1, Vec2 v2) {
        m_vertex1.set(v1);
        m_vertex2.set(v2);
        m_hasVertex0 = false;
        m_hasVertex3 = false;
    }
}
