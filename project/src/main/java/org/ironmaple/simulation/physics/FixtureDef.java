package org.ironmaple.simulation.physics;

/**
 *
 *
 * <h2>Fixture definition for Box2D.</h2>
 *
 * <p>Contains the parameters needed to create a fixture.
 */
public class FixtureDef {
    /** The shape. */
    public Shape shape = null;

    /** User data pointer. */
    public Object userData = null;

    /** The friction coefficient, usually in the range [0,1]. */
    public float friction = 0.2f;

    /** The restitution (elasticity), usually in the range [0,1]. */
    public float restitution = 0;

    /** The restitution velocity threshold. */
    public float restitutionThreshold = 1.0f;

    /** The density, usually in kg/m^2. */
    public float density = 0;

    /** A sensor shape collects contact information but never generates a collision response. */
    public boolean isSensor = false;

    /** Contact filtering data. */
    public Filter filter = new Filter();

    /**
     *
     *
     * <h2>Contact filter data.</h2>
     */
    public static class Filter {
        /** The collision category bits. */
        public int categoryBits = 0x0001;

        /** The collision mask bits. */
        public int maskBits = 0xFFFF;

        /** The collision group index. */
        public int groupIndex = 0;
    }
}
