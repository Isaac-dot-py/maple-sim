package org.ironmaple.simulation.physics;

/**
 *
 *
 * <h2>Contact impulse.</h2>
 *
 * <p>Contains impulse information from a contact.
 */
public class ContactImpulse {
    /** Normal impulses. */
    public float[] normalImpulses = new float[2];

    /** Tangent impulses. */
    public float[] tangentImpulses = new float[2];

    /** Number of impulse values. */
    public int count = 0;
}
