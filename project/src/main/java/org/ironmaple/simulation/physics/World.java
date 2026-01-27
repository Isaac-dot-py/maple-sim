package org.ironmaple.simulation.physics;

import java.util.HashMap;
import java.util.Map;

/**
 *
 *
 * <h2>Physics World for Box2D.</h2>
 *
 * <p>The world class manages all physics entities, dynamic simulation, and queries.
 */
public class World {
    static {
        Box2DLoader.load();
    }

    /** Native pointer to b2World. */
    private long nativePtr;

    /** Body list head. */
    private Body bodyList;

    /** Contact listener. */
    private ContactListener contactListener;

    /** Map from native body pointers to Java Body objects. */
    private final Map<Long, Body> bodyMap = new HashMap<>();

    /** Map from native fixture pointers to Java Fixture objects. */
    private final Map<Long, Fixture> fixtureMap = new HashMap<>();

    /**
     *
     *
     * <h2>Create a physics world.</h2>
     *
     * @param gravity The world gravity vector
     */
    public World(Vec2 gravity) {
        nativePtr = nativeCreate(gravity.x, gravity.y);
    }

    /**
     *
     *
     * <h2>Destroy the world and all bodies.</h2>
     */
    public void dispose() {
        if (nativePtr != 0) {
            nativeDestroy(nativePtr);
            nativePtr = 0;
        }
    }

    /**
     *
     *
     * <h2>Create a body in the world.</h2>
     *
     * @param def The body definition
     * @return The created body
     */
    public Body createBody(BodyDef def) {
        long bodyPtr = nativeCreateBody(
                nativePtr,
                def.type.getValue(),
                def.position.x,
                def.position.y,
                def.angle,
                def.linearVelocity.x,
                def.linearVelocity.y,
                def.angularVelocity,
                def.linearDamping,
                def.angularDamping,
                def.allowSleep,
                def.awake,
                def.fixedRotation,
                def.bullet,
                def.active,
                def.gravityScale);

        Body body = new Body(bodyPtr, this);
        body.userData = def.userData;
        body.next = bodyList;
        bodyList = body;
        bodyMap.put(bodyPtr, body);
        return body;
    }

    /**
     *
     *
     * <h2>Destroy a body.</h2>
     *
     * @param body The body to destroy
     */
    public void destroyBody(Body body) {
        // Remove from linked list
        if (bodyList == body) {
            bodyList = body.next;
        } else {
            Body b = bodyList;
            while (b != null && b.next != body) {
                b = b.next;
            }
            if (b != null) {
                b.next = body.next;
            }
        }
        bodyMap.remove(body.nativePtr);
        nativeDestroyBody(nativePtr, body.nativePtr);
    }

    /**
     *
     *
     * <h2>Step the simulation.</h2>
     *
     * @param timeStep The time step in seconds
     * @param velocityIterations Number of velocity constraint solver iterations
     * @param positionIterations Number of position constraint solver iterations
     */
    public void step(float timeStep, int velocityIterations, int positionIterations) {
        nativeStep(nativePtr, timeStep, velocityIterations, positionIterations);
    }

    /**
     *
     *
     * <h2>Set the contact listener.</h2>
     *
     * @param listener The contact listener
     */
    public void setContactListener(ContactListener listener) {
        this.contactListener = listener;
        nativeSetContactListener(nativePtr, listener != null);
    }

    /**
     *
     *
     * <h2>Get the contact listener.</h2>
     *
     * @return The contact listener
     */
    public ContactListener getContactListener() {
        return contactListener;
    }

    /**
     *
     *
     * <h2>Set the gravity.</h2>
     *
     * @param gravity The gravity vector
     */
    public void setGravity(Vec2 gravity) {
        nativeSetGravity(nativePtr, gravity.x, gravity.y);
    }

    /**
     *
     *
     * <h2>Get the gravity.</h2>
     *
     * @return The gravity vector
     */
    public Vec2 getGravity() {
        float[] data = nativeGetGravity(nativePtr);
        return new Vec2(data[0], data[1]);
    }

    /**
     *
     *
     * <h2>Is the world locked?</h2>
     *
     * @return True if locked (during step)
     */
    public boolean isLocked() {
        return nativeIsLocked(nativePtr);
    }

    /**
     *
     *
     * <h2>Set automatic clearing of forces after each step.</h2>
     *
     * @param flag True to auto-clear
     */
    public void setAutoClearForces(boolean flag) {
        nativeSetAutoClearForces(nativePtr, flag);
    }

    /**
     *
     *
     * <h2>Get automatic clearing of forces.</h2>
     *
     * @return True if auto-clearing
     */
    public boolean getAutoClearForces() {
        return nativeGetAutoClearForces(nativePtr);
    }

    /**
     *
     *
     * <h2>Clear all forces.</h2>
     */
    public void clearForces() {
        nativeClearForces(nativePtr);
    }

    /**
     *
     *
     * <h2>Get the body list.</h2>
     *
     * @return The first body
     */
    public Body getBodyList() {
        return bodyList;
    }

    /**
     *
     *
     * <h2>Get the body count.</h2>
     *
     * @return The number of bodies
     */
    public int getBodyCount() {
        return nativeGetBodyCount(nativePtr);
    }

    /**
     *
     *
     * <h2>Get a body by native pointer.</h2>
     *
     * @param ptr The native pointer
     * @return The body, or null
     */
    Body getBody(long ptr) {
        return bodyMap.get(ptr);
    }

    /**
     *
     *
     * <h2>Register a fixture in the map.</h2>
     *
     * @param ptr The native pointer
     * @param fixture The fixture
     */
    void registerFixture(long ptr, Fixture fixture) {
        fixtureMap.put(ptr, fixture);
    }

    /**
     *
     *
     * <h2>Get a fixture by native pointer.</h2>
     *
     * @param ptr The native pointer
     * @return The fixture, or null
     */
    Fixture getFixture(long ptr) {
        return fixtureMap.get(ptr);
    }

    // Callback from native code for beginContact
    @SuppressWarnings("unused")
    private void beginContactCallback(long contactPtr, long fixtureAPtr, long fixtureBPtr) {
        if (contactListener != null) {
            Contact contact = new Contact(contactPtr);
            contact.fixtureA = fixtureMap.get(fixtureAPtr);
            contact.fixtureB = fixtureMap.get(fixtureBPtr);
            contactListener.beginContact(contact);
        }
    }

    // Callback from native code for endContact
    @SuppressWarnings("unused")
    private void endContactCallback(long contactPtr, long fixtureAPtr, long fixtureBPtr) {
        if (contactListener != null) {
            Contact contact = new Contact(contactPtr);
            contact.fixtureA = fixtureMap.get(fixtureAPtr);
            contact.fixtureB = fixtureMap.get(fixtureBPtr);
            contactListener.endContact(contact);
        }
    }

    // Callback from native code for preSolve
    @SuppressWarnings("unused")
    private void preSolveCallback(long contactPtr, long fixtureAPtr, long fixtureBPtr) {
        if (contactListener != null) {
            Contact contact = new Contact(contactPtr);
            contact.fixtureA = fixtureMap.get(fixtureAPtr);
            contact.fixtureB = fixtureMap.get(fixtureBPtr);
            contactListener.preSolve(contact, new Manifold());
        }
    }

    // Callback from native code for postSolve
    @SuppressWarnings("unused")
    private void postSolveCallback(
            long contactPtr, long fixtureAPtr, long fixtureBPtr, float[] normalImpulses, float[] tangentImpulses) {
        if (contactListener != null) {
            Contact contact = new Contact(contactPtr);
            contact.fixtureA = fixtureMap.get(fixtureAPtr);
            contact.fixtureB = fixtureMap.get(fixtureBPtr);
            ContactImpulse impulse = new ContactImpulse();
            impulse.normalImpulses = normalImpulses;
            impulse.tangentImpulses = tangentImpulses;
            impulse.count = normalImpulses.length;
            contactListener.postSolve(contact, impulse);
        }
    }

    // Native methods
    private static native long nativeCreate(float gravityX, float gravityY);

    private static native void nativeDestroy(long ptr);

    private static native long nativeCreateBody(
            long worldPtr,
            int type,
            float px,
            float py,
            float angle,
            float vx,
            float vy,
            float angularVelocity,
            float linearDamping,
            float angularDamping,
            boolean allowSleep,
            boolean awake,
            boolean fixedRotation,
            boolean bullet,
            boolean active,
            float gravityScale);

    private static native void nativeDestroyBody(long worldPtr, long bodyPtr);

    private static native void nativeStep(long ptr, float timeStep, int velocityIterations, int positionIterations);

    private static native void nativeSetContactListener(long ptr, boolean hasListener);

    private static native void nativeSetGravity(long ptr, float x, float y);

    private static native float[] nativeGetGravity(long ptr);

    private static native boolean nativeIsLocked(long ptr);

    private static native void nativeSetAutoClearForces(long ptr, boolean flag);

    private static native boolean nativeGetAutoClearForces(long ptr);

    private static native void nativeClearForces(long ptr);

    private static native int nativeGetBodyCount(long ptr);
}
