package org.ironmaple.simulation.physics;

/**
 *
 *
 * <h2>Polygon shape for Box2D.</h2>
 *
 * <p>A convex polygon shape. Vertices are stored counterclockwise.
 */
public class PolygonShape extends Shape {
    /** Maximum number of vertices per polygon. */
    public static final int MAX_POLYGON_VERTICES = 8;

    /** The vertices of the polygon. */
    public Vec2[] m_vertices = new Vec2[MAX_POLYGON_VERTICES];

    /** The normals of the polygon edges. */
    public Vec2[] m_normals = new Vec2[MAX_POLYGON_VERTICES];

    /** The centroid of the polygon. */
    public Vec2 m_centroid = new Vec2();

    /** The number of vertices. */
    public int m_count = 0;

    /** The radius for collision detection. */
    public float m_radius = 0.01f; // b2_polygonRadius

    /**
     *
     *
     * <h2>Creates an empty polygon shape.</h2>
     */
    public PolygonShape() {
        for (int i = 0; i < MAX_POLYGON_VERTICES; i++) {
            m_vertices[i] = new Vec2();
            m_normals[i] = new Vec2();
        }
    }

    @Override
    public ShapeType getType() {
        return ShapeType.POLYGON;
    }

    @Override
    public int getChildCount() {
        return 1;
    }

    @Override
    public PolygonShape clone() {
        PolygonShape shape = new PolygonShape();
        shape.m_count = this.m_count;
        shape.m_radius = this.m_radius;
        shape.m_centroid = new Vec2(this.m_centroid);
        for (int i = 0; i < m_count; i++) {
            shape.m_vertices[i] = new Vec2(this.m_vertices[i]);
            shape.m_normals[i] = new Vec2(this.m_normals[i]);
        }
        return shape;
    }

    /**
     *
     *
     * <h2>Set the shape as a box.</h2>
     *
     * <p>Creates a rectangle centered at the origin.
     *
     * @param hx The half-width in the x direction
     * @param hy The half-height in the y direction
     */
    public void setAsBox(float hx, float hy) {
        m_count = 4;
        m_vertices[0].set(-hx, -hy);
        m_vertices[1].set(hx, -hy);
        m_vertices[2].set(hx, hy);
        m_vertices[3].set(-hx, hy);
        m_normals[0].set(0, -1);
        m_normals[1].set(1, 0);
        m_normals[2].set(0, 1);
        m_normals[3].set(-1, 0);
        m_centroid.setZero();
    }

    /**
     *
     *
     * <h2>Set the shape as a box with a center and angle.</h2>
     *
     * @param hx The half-width in the x direction
     * @param hy The half-height in the y direction
     * @param center The center of the box in local coordinates
     * @param angle The rotation angle in radians
     */
    public void setAsBox(float hx, float hy, Vec2 center, float angle) {
        m_count = 4;
        m_vertices[0].set(-hx, -hy);
        m_vertices[1].set(hx, -hy);
        m_vertices[2].set(hx, hy);
        m_vertices[3].set(-hx, hy);
        m_normals[0].set(0, -1);
        m_normals[1].set(1, 0);
        m_normals[2].set(0, 1);
        m_normals[3].set(-1, 0);
        m_centroid.set(center);

        // Transform vertices and normals
        float c = (float) Math.cos(angle);
        float s = (float) Math.sin(angle);

        for (int i = 0; i < m_count; i++) {
            float vx = m_vertices[i].x;
            float vy = m_vertices[i].y;
            m_vertices[i].x = center.x + c * vx - s * vy;
            m_vertices[i].y = center.y + s * vx + c * vy;

            float nx = m_normals[i].x;
            float ny = m_normals[i].y;
            m_normals[i].x = c * nx - s * ny;
            m_normals[i].y = s * nx + c * ny;
        }
    }

    /**
     *
     *
     * <h2>Set the vertices of the polygon.</h2>
     *
     * @param vertices The vertices array
     * @param count The number of vertices
     */
    public void set(Vec2[] vertices, int count) {
        if (count < 3) {
            throw new IllegalArgumentException("Polygon must have at least 3 vertices");
        }
        if (count > MAX_POLYGON_VERTICES) {
            throw new IllegalArgumentException("Too many vertices: " + count + " > " + MAX_POLYGON_VERTICES);
        }

        m_count = count;

        // Copy vertices
        for (int i = 0; i < count; i++) {
            m_vertices[i].set(vertices[i]);
        }

        // Compute normals
        for (int i = 0; i < count; i++) {
            int i1 = i;
            int i2 = i + 1 < count ? i + 1 : 0;
            Vec2 edge = Vec2.sub(m_vertices[i2], m_vertices[i1]);
            // Cross product with z-axis gives perpendicular
            m_normals[i].set(edge.y, -edge.x);
            m_normals[i].normalize();
        }

        // Compute centroid
        computeCentroid();
    }

    private void computeCentroid() {
        m_centroid.setZero();
        float area = 0;

        Vec2 pRef = new Vec2();
        final float inv3 = 1.0f / 3.0f;

        for (int i = 0; i < m_count; i++) {
            Vec2 p1 = pRef;
            Vec2 p2 = m_vertices[i];
            Vec2 p3 = i + 1 < m_count ? m_vertices[i + 1] : m_vertices[0];

            Vec2 e1 = Vec2.sub(p2, p1);
            Vec2 e2 = Vec2.sub(p3, p1);

            float D = e1.x * e2.y - e1.y * e2.x;
            float triangleArea = 0.5f * D;
            area += triangleArea;

            m_centroid.x += triangleArea * inv3 * (p1.x + p2.x + p3.x);
            m_centroid.y += triangleArea * inv3 * (p1.y + p2.y + p3.y);
        }

        if (area > 1e-10f) {
            m_centroid.x /= area;
            m_centroid.y /= area;
        }
    }

    /**
     *
     *
     * <h2>Get the number of vertices.</h2>
     *
     * @return The vertex count
     */
    public int getVertexCount() {
        return m_count;
    }

    /**
     *
     *
     * <h2>Get a vertex by index.</h2>
     *
     * @param index The vertex index
     * @return The vertex
     */
    public Vec2 getVertex(int index) {
        return m_vertices[index];
    }
}
