package org.ironmaple.simulation.physics;

/**
 *
 *
 * <h2>Contact listener interface.</h2>
 *
 * <p>Implement this interface to receive collision events.
 */
public interface ContactListener {
    /**
     *
     *
     * <h2>Called when two fixtures begin to touch.</h2>
     *
     * @param contact The contact
     */
    void beginContact(Contact contact);

    /**
     *
     *
     * <h2>Called when two fixtures cease to touch.</h2>
     *
     * @param contact The contact
     */
    void endContact(Contact contact);

    /**
     *
     *
     * <h2>Called before a contact is solved.</h2>
     *
     * <p>This gives you a chance to disable the contact.
     *
     * @param contact The contact
     * @param oldManifold The old manifold
     */
    void preSolve(Contact contact, Manifold oldManifold);

    /**
     *
     *
     * <h2>Called after a contact is solved.</h2>
     *
     * @param contact The contact
     * @param impulse The contact impulse
     */
    void postSolve(Contact contact, ContactImpulse impulse);
}
