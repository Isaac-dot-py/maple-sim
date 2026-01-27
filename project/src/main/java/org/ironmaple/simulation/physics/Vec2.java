package org.ironmaple.simulation.physics;

/**
 *
 *
 * <h2>2D Vector for Box2D.</h2>
 *
 * <p>A simple 2D vector class that mirrors Box2D's b2Vec2.
 */
public class Vec2 {
    public float x;
    public float y;

    /**
     *
     *
     * <h2>Creates a zero vector.</h2>
     */
    public Vec2() {
        this.x = 0;
        this.y = 0;
    }

    /**
     *
     *
     * <h2>Creates a vector with the given components.</h2>
     *
     * @param x The x component
     * @param y The y component
     */
    public Vec2(float x, float y) {
        this.x = x;
        this.y = y;
    }

    /**
     *
     *
     * <h2>Copy constructor.</h2>
     *
     * @param v The vector to copy
     */
    public Vec2(Vec2 v) {
        this.x = v.x;
        this.y = v.y;
    }

    /**
     *
     *
     * <h2>Set the vector components.</h2>
     *
     * @param x The x component
     * @param y The y component
     * @return This vector for chaining
     */
    public Vec2 set(float x, float y) {
        this.x = x;
        this.y = y;
        return this;
    }

    /**
     *
     *
     * <h2>Set this vector to another vector.</h2>
     *
     * @param v The vector to copy
     * @return This vector for chaining
     */
    public Vec2 set(Vec2 v) {
        this.x = v.x;
        this.y = v.y;
        return this;
    }

    /**
     *
     *
     * <h2>Add a vector to this vector.</h2>
     *
     * @param v The vector to add
     * @return This vector for chaining
     */
    public Vec2 addLocal(Vec2 v) {
        this.x += v.x;
        this.y += v.y;
        return this;
    }

    /**
     *
     *
     * <h2>Subtract a vector from this vector.</h2>
     *
     * @param v The vector to subtract
     * @return This vector for chaining
     */
    public Vec2 subLocal(Vec2 v) {
        this.x -= v.x;
        this.y -= v.y;
        return this;
    }

    /**
     *
     *
     * <h2>Multiply this vector by a scalar.</h2>
     *
     * @param s The scalar
     * @return This vector for chaining
     */
    public Vec2 mulLocal(float s) {
        this.x *= s;
        this.y *= s;
        return this;
    }

    /**
     *
     *
     * <h2>Add two vectors.</h2>
     *
     * @param a First vector
     * @param b Second vector
     * @return The sum
     */
    public static Vec2 add(Vec2 a, Vec2 b) {
        return new Vec2(a.x + b.x, a.y + b.y);
    }

    /**
     *
     *
     * <h2>Subtract two vectors.</h2>
     *
     * @param a First vector
     * @param b Second vector
     * @return a - b
     */
    public static Vec2 sub(Vec2 a, Vec2 b) {
        return new Vec2(a.x - b.x, a.y - b.y);
    }

    /**
     *
     *
     * <h2>Multiply a vector by a scalar.</h2>
     *
     * @param v The vector
     * @param s The scalar
     * @return The scaled vector
     */
    public static Vec2 mul(Vec2 v, float s) {
        return new Vec2(v.x * s, v.y * s);
    }

    /**
     *
     *
     * <h2>Get the length of this vector.</h2>
     *
     * @return The length
     */
    public float length() {
        return (float) Math.sqrt(x * x + y * y);
    }

    /**
     *
     *
     * <h2>Get the squared length of this vector.</h2>
     *
     * @return The squared length
     */
    public float lengthSquared() {
        return x * x + y * y;
    }

    /**
     *
     *
     * <h2>Normalize this vector.</h2>
     *
     * @return The original length
     */
    public float normalize() {
        float len = length();
        if (len < 1e-10f) {
            return 0;
        }
        float invLen = 1.0f / len;
        x *= invLen;
        y *= invLen;
        return len;
    }

    /**
     *
     *
     * <h2>Set this vector to zero.</h2>
     */
    public void setZero() {
        x = 0;
        y = 0;
    }

    /**
     *
     *
     * <h2>Clone this vector.</h2>
     *
     * @return A copy of this vector
     */
    @Override
    public Vec2 clone() {
        return new Vec2(x, y);
    }

    @Override
    public String toString() {
        return "Vec2(" + x + ", " + y + ")";
    }
}
