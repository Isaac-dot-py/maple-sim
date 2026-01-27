// Box2D JNI Wrapper Implementation
// Provides JNI bindings for Box2D C++ library to Java

#include <jni.h>
#include <box2d/box2d.h>
#include <map>
#include <mutex>

// Cache for JNI references
static jclass worldClass = nullptr;
static jmethodID beginContactMethod = nullptr;
static jmethodID endContactMethod = nullptr;
static jmethodID preSolveMethod = nullptr;
static jmethodID postSolveMethod = nullptr;

// Contact listener that forwards to Java
class JniContactListener : public b2ContactListener {
public:
    JNIEnv* env;
    jobject worldObject;
    
    JniContactListener(JNIEnv* e, jobject wo) : env(e), worldObject(wo) {}
    
    void BeginContact(b2Contact* contact) override {
        if (beginContactMethod) {
            b2Fixture* fixtureA = contact->GetFixtureA();
            b2Fixture* fixtureB = contact->GetFixtureB();
            env->CallVoidMethod(worldObject, beginContactMethod,
                (jlong)contact, (jlong)fixtureA, (jlong)fixtureB);
        }
    }
    
    void EndContact(b2Contact* contact) override {
        if (endContactMethod) {
            b2Fixture* fixtureA = contact->GetFixtureA();
            b2Fixture* fixtureB = contact->GetFixtureB();
            env->CallVoidMethod(worldObject, endContactMethod,
                (jlong)contact, (jlong)fixtureA, (jlong)fixtureB);
        }
    }
    
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override {
        if (preSolveMethod) {
            b2Fixture* fixtureA = contact->GetFixtureA();
            b2Fixture* fixtureB = contact->GetFixtureB();
            env->CallVoidMethod(worldObject, preSolveMethod,
                (jlong)contact, (jlong)fixtureA, (jlong)fixtureB);
        }
    }
    
    void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) override {
        if (postSolveMethod) {
            b2Fixture* fixtureA = contact->GetFixtureA();
            b2Fixture* fixtureB = contact->GetFixtureB();
            
            jfloatArray normalImpulses = env->NewFloatArray(impulse->count);
            jfloatArray tangentImpulses = env->NewFloatArray(impulse->count);
            env->SetFloatArrayRegion(normalImpulses, 0, impulse->count, impulse->normalImpulses);
            env->SetFloatArrayRegion(tangentImpulses, 0, impulse->count, impulse->tangentImpulses);
            
            env->CallVoidMethod(worldObject, postSolveMethod,
                (jlong)contact, (jlong)fixtureA, (jlong)fixtureB,
                normalImpulses, tangentImpulses);
            
            env->DeleteLocalRef(normalImpulses);
            env->DeleteLocalRef(tangentImpulses);
        }
    }
};

// Store contact listeners per world
static std::map<b2World*, JniContactListener*> contactListeners;
static std::mutex listenerMutex;

extern "C" {

// ============ World Functions ============

JNIEXPORT jlong JNICALL Java_org_ironmaple_simulation_physics_World_nativeCreate
    (JNIEnv* env, jclass cls, jfloat gravityX, jfloat gravityY) {
    b2Vec2 gravity(gravityX, gravityY);
    b2World* world = new b2World(gravity);
    return (jlong)world;
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_World_nativeDestroy
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2World* world = (b2World*)ptr;
    
    // Clean up contact listener
    {
        std::lock_guard<std::mutex> lock(listenerMutex);
        auto it = contactListeners.find(world);
        if (it != contactListeners.end()) {
            delete it->second;
            contactListeners.erase(it);
        }
    }
    
    delete world;
}

JNIEXPORT jlong JNICALL Java_org_ironmaple_simulation_physics_World_nativeCreateBody
    (JNIEnv* env, jclass cls, jlong worldPtr,
     jint type, jfloat px, jfloat py, jfloat angle,
     jfloat vx, jfloat vy, jfloat angularVelocity,
     jfloat linearDamping, jfloat angularDamping,
     jboolean allowSleep, jboolean awake,
     jboolean fixedRotation, jboolean bullet,
     jboolean active, jfloat gravityScale) {
    
    b2World* world = (b2World*)worldPtr;
    
    b2BodyDef bodyDef;
    bodyDef.type = (b2BodyType)type;
    bodyDef.position.Set(px, py);
    bodyDef.angle = angle;
    bodyDef.linearVelocity.Set(vx, vy);
    bodyDef.angularVelocity = angularVelocity;
    bodyDef.linearDamping = linearDamping;
    bodyDef.angularDamping = angularDamping;
    bodyDef.allowSleep = allowSleep;
    bodyDef.awake = awake;
    bodyDef.fixedRotation = fixedRotation;
    bodyDef.bullet = bullet;
    bodyDef.enabled = active;
    bodyDef.gravityScale = gravityScale;
    
    b2Body* body = world->CreateBody(&bodyDef);
    return (jlong)body;
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_World_nativeDestroyBody
    (JNIEnv* env, jclass cls, jlong worldPtr, jlong bodyPtr) {
    b2World* world = (b2World*)worldPtr;
    b2Body* body = (b2Body*)bodyPtr;
    world->DestroyBody(body);
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_World_nativeStep
    (JNIEnv* env, jclass cls, jlong ptr, jfloat timeStep, jint velocityIterations, jint positionIterations) {
    b2World* world = (b2World*)ptr;
    world->Step(timeStep, velocityIterations, positionIterations);
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_World_nativeSetContactListener
    (JNIEnv* env, jobject obj, jlong ptr, jboolean hasListener) {
    b2World* world = (b2World*)ptr;
    
    std::lock_guard<std::mutex> lock(listenerMutex);
    
    // Remove existing listener
    auto it = contactListeners.find(world);
    if (it != contactListeners.end()) {
        delete it->second;
        contactListeners.erase(it);
    }
    
    if (hasListener) {
        // Cache method IDs if not already done
        if (!worldClass) {
            jclass localClass = env->GetObjectClass(obj);
            worldClass = (jclass)env->NewGlobalRef(localClass);
            beginContactMethod = env->GetMethodID(worldClass, "beginContactCallback", "(JJJ)V");
            endContactMethod = env->GetMethodID(worldClass, "endContactCallback", "(JJJ)V");
            preSolveMethod = env->GetMethodID(worldClass, "preSolveCallback", "(JJJ)V");
            postSolveMethod = env->GetMethodID(worldClass, "postSolveCallback", "(JJJ[F[F)V");
        }
        
        JniContactListener* listener = new JniContactListener(env, obj);
        contactListeners[world] = listener;
        world->SetContactListener(listener);
    } else {
        world->SetContactListener(nullptr);
    }
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_World_nativeSetGravity
    (JNIEnv* env, jclass cls, jlong ptr, jfloat x, jfloat y) {
    b2World* world = (b2World*)ptr;
    world->SetGravity(b2Vec2(x, y));
}

JNIEXPORT jfloatArray JNICALL Java_org_ironmaple_simulation_physics_World_nativeGetGravity
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2World* world = (b2World*)ptr;
    b2Vec2 gravity = world->GetGravity();
    jfloatArray result = env->NewFloatArray(2);
    float data[2] = {gravity.x, gravity.y};
    env->SetFloatArrayRegion(result, 0, 2, data);
    return result;
}

JNIEXPORT jboolean JNICALL Java_org_ironmaple_simulation_physics_World_nativeIsLocked
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2World* world = (b2World*)ptr;
    return world->IsLocked();
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_World_nativeSetAutoClearForces
    (JNIEnv* env, jclass cls, jlong ptr, jboolean flag) {
    b2World* world = (b2World*)ptr;
    world->SetAutoClearForces(flag);
}

JNIEXPORT jboolean JNICALL Java_org_ironmaple_simulation_physics_World_nativeGetAutoClearForces
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2World* world = (b2World*)ptr;
    return world->GetAutoClearForces();
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_World_nativeClearForces
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2World* world = (b2World*)ptr;
    world->ClearForces();
}

JNIEXPORT jint JNICALL Java_org_ironmaple_simulation_physics_World_nativeGetBodyCount
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2World* world = (b2World*)ptr;
    return world->GetBodyCount();
}

// ============ Body Functions ============

// Helper to create fixture from FixtureDef
static b2FixtureDef createFixtureDef(JNIEnv* env, jobject def, b2Shape** outShape) {
    b2FixtureDef fixtureDef;
    
    jclass defClass = env->GetObjectClass(def);
    jfieldID shapeField = env->GetFieldID(defClass, "shape", "Lorg/ironmaple/simulation/physics/Shape;");
    jfieldID frictionField = env->GetFieldID(defClass, "friction", "F");
    jfieldID restitutionField = env->GetFieldID(defClass, "restitution", "F");
    jfieldID densityField = env->GetFieldID(defClass, "density", "F");
    jfieldID isSensorField = env->GetFieldID(defClass, "isSensor", "Z");
    
    fixtureDef.friction = env->GetFloatField(def, frictionField);
    fixtureDef.restitution = env->GetFloatField(def, restitutionField);
    fixtureDef.density = env->GetFloatField(def, densityField);
    fixtureDef.isSensor = env->GetBooleanField(def, isSensorField);
    
    // Get the shape
    jobject shapeObj = env->GetObjectField(def, shapeField);
    if (shapeObj) {
        jclass shapeClass = env->GetObjectClass(shapeObj);
        jmethodID getTypeMethod = env->GetMethodID(shapeClass, "getType", "()Lorg/ironmaple/simulation/physics/Shape$ShapeType;");
        jobject typeObj = env->CallObjectMethod(shapeObj, getTypeMethod);
        
        jclass typeClass = env->GetObjectClass(typeObj);
        jmethodID ordinalMethod = env->GetMethodID(typeClass, "ordinal", "()I");
        jint shapeType = env->CallIntMethod(typeObj, ordinalMethod);
        
        switch (shapeType) {
            case 0: { // CIRCLE
                jclass circleClass = env->FindClass("org/ironmaple/simulation/physics/CircleShape");
                jfieldID radiusField = env->GetFieldID(circleClass, "m_radius", "F");
                jfieldID pField = env->GetFieldID(circleClass, "m_p", "Lorg/ironmaple/simulation/physics/Vec2;");
                
                float radius = env->GetFloatField(shapeObj, radiusField);
                jobject pObj = env->GetObjectField(shapeObj, pField);
                
                jclass vec2Class = env->GetObjectClass(pObj);
                jfieldID xField = env->GetFieldID(vec2Class, "x", "F");
                jfieldID yField = env->GetFieldID(vec2Class, "y", "F");
                float px = env->GetFloatField(pObj, xField);
                float py = env->GetFloatField(pObj, yField);
                
                b2CircleShape* circle = new b2CircleShape();
                circle->m_radius = radius;
                circle->m_p.Set(px, py);
                *outShape = circle;
                fixtureDef.shape = circle;
                break;
            }
            case 1: { // EDGE
                jclass edgeClass = env->FindClass("org/ironmaple/simulation/physics/EdgeShape");
                jfieldID v1Field = env->GetFieldID(edgeClass, "m_vertex1", "Lorg/ironmaple/simulation/physics/Vec2;");
                jfieldID v2Field = env->GetFieldID(edgeClass, "m_vertex2", "Lorg/ironmaple/simulation/physics/Vec2;");
                
                jobject v1Obj = env->GetObjectField(shapeObj, v1Field);
                jobject v2Obj = env->GetObjectField(shapeObj, v2Field);
                
                jclass vec2Class = env->FindClass("org/ironmaple/simulation/physics/Vec2");
                jfieldID xField = env->GetFieldID(vec2Class, "x", "F");
                jfieldID yField = env->GetFieldID(vec2Class, "y", "F");
                
                float v1x = env->GetFloatField(v1Obj, xField);
                float v1y = env->GetFloatField(v1Obj, yField);
                float v2x = env->GetFloatField(v2Obj, xField);
                float v2y = env->GetFloatField(v2Obj, yField);
                
                b2EdgeShape* edge = new b2EdgeShape();
                edge->SetTwoSided(b2Vec2(v1x, v1y), b2Vec2(v2x, v2y));
                *outShape = edge;
                fixtureDef.shape = edge;
                break;
            }
            case 2: { // POLYGON
                jclass polyClass = env->FindClass("org/ironmaple/simulation/physics/PolygonShape");
                jfieldID countField = env->GetFieldID(polyClass, "m_count", "I");
                jfieldID verticesField = env->GetFieldID(polyClass, "m_vertices", "[Lorg/ironmaple/simulation/physics/Vec2;");
                
                int count = env->GetIntField(shapeObj, countField);
                jobjectArray verticesArray = (jobjectArray)env->GetObjectField(shapeObj, verticesField);
                
                jclass vec2Class = env->FindClass("org/ironmaple/simulation/physics/Vec2");
                jfieldID xField = env->GetFieldID(vec2Class, "x", "F");
                jfieldID yField = env->GetFieldID(vec2Class, "y", "F");
                
                b2Vec2* vertices = new b2Vec2[count];
                for (int i = 0; i < count; i++) {
                    jobject vObj = env->GetObjectArrayElement(verticesArray, i);
                    vertices[i].x = env->GetFloatField(vObj, xField);
                    vertices[i].y = env->GetFloatField(vObj, yField);
                }
                
                b2PolygonShape* polygon = new b2PolygonShape();
                polygon->Set(vertices, count);
                delete[] vertices;
                
                *outShape = polygon;
                fixtureDef.shape = polygon;
                break;
            }
            default:
                *outShape = nullptr;
                break;
        }
    }
    
    return fixtureDef;
}

JNIEXPORT jlong JNICALL Java_org_ironmaple_simulation_physics_Body_nativeCreateFixture
    (JNIEnv* env, jclass cls, jlong bodyPtr, jobject def) {
    b2Body* body = (b2Body*)bodyPtr;
    b2Shape* shape = nullptr;
    b2FixtureDef fixtureDef = createFixtureDef(env, def, &shape);
    
    b2Fixture* fixture = body->CreateFixture(&fixtureDef);
    
    // Clean up temporary shape
    if (shape) delete shape;
    
    return (jlong)fixture;
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeDestroyFixture
    (JNIEnv* env, jclass cls, jlong bodyPtr, jlong fixturePtr) {
    b2Body* body = (b2Body*)bodyPtr;
    b2Fixture* fixture = (b2Fixture*)fixturePtr;
    body->DestroyFixture(fixture);
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeSetTransform
    (JNIEnv* env, jclass cls, jlong ptr, jfloat x, jfloat y, jfloat angle) {
    b2Body* body = (b2Body*)ptr;
    body->SetTransform(b2Vec2(x, y), angle);
}

JNIEXPORT jfloatArray JNICALL Java_org_ironmaple_simulation_physics_Body_nativeGetTransform
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    const b2Transform& xf = body->GetTransform();
    jfloatArray result = env->NewFloatArray(4);
    float data[4] = {xf.p.x, xf.p.y, xf.q.c, xf.q.s};
    env->SetFloatArrayRegion(result, 0, 4, data);
    return result;
}

JNIEXPORT jfloatArray JNICALL Java_org_ironmaple_simulation_physics_Body_nativeGetPosition
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    const b2Vec2& pos = body->GetPosition();
    jfloatArray result = env->NewFloatArray(2);
    float data[2] = {pos.x, pos.y};
    env->SetFloatArrayRegion(result, 0, 2, data);
    return result;
}

JNIEXPORT jfloat JNICALL Java_org_ironmaple_simulation_physics_Body_nativeGetAngle
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    return body->GetAngle();
}

JNIEXPORT jfloatArray JNICALL Java_org_ironmaple_simulation_physics_Body_nativeGetWorldCenter
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    const b2Vec2& center = body->GetWorldCenter();
    jfloatArray result = env->NewFloatArray(2);
    float data[2] = {center.x, center.y};
    env->SetFloatArrayRegion(result, 0, 2, data);
    return result;
}

JNIEXPORT jfloatArray JNICALL Java_org_ironmaple_simulation_physics_Body_nativeGetLocalCenter
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    const b2Vec2& center = body->GetLocalCenter();
    jfloatArray result = env->NewFloatArray(2);
    float data[2] = {center.x, center.y};
    env->SetFloatArrayRegion(result, 0, 2, data);
    return result;
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeSetLinearVelocity
    (JNIEnv* env, jclass cls, jlong ptr, jfloat vx, jfloat vy) {
    b2Body* body = (b2Body*)ptr;
    body->SetLinearVelocity(b2Vec2(vx, vy));
}

JNIEXPORT jfloatArray JNICALL Java_org_ironmaple_simulation_physics_Body_nativeGetLinearVelocity
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    const b2Vec2& vel = body->GetLinearVelocity();
    jfloatArray result = env->NewFloatArray(2);
    float data[2] = {vel.x, vel.y};
    env->SetFloatArrayRegion(result, 0, 2, data);
    return result;
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeSetAngularVelocity
    (JNIEnv* env, jclass cls, jlong ptr, jfloat omega) {
    b2Body* body = (b2Body*)ptr;
    body->SetAngularVelocity(omega);
}

JNIEXPORT jfloat JNICALL Java_org_ironmaple_simulation_physics_Body_nativeGetAngularVelocity
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    return body->GetAngularVelocity();
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeApplyForce
    (JNIEnv* env, jclass cls, jlong ptr, jfloat fx, jfloat fy, jfloat px, jfloat py, jboolean wake) {
    b2Body* body = (b2Body*)ptr;
    body->ApplyForce(b2Vec2(fx, fy), b2Vec2(px, py), wake);
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeApplyForceToCenter
    (JNIEnv* env, jclass cls, jlong ptr, jfloat fx, jfloat fy, jboolean wake) {
    b2Body* body = (b2Body*)ptr;
    body->ApplyForceToCenter(b2Vec2(fx, fy), wake);
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeApplyTorque
    (JNIEnv* env, jclass cls, jlong ptr, jfloat torque, jboolean wake) {
    b2Body* body = (b2Body*)ptr;
    body->ApplyTorque(torque, wake);
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeApplyLinearImpulse
    (JNIEnv* env, jclass cls, jlong ptr, jfloat ix, jfloat iy, jfloat px, jfloat py, jboolean wake) {
    b2Body* body = (b2Body*)ptr;
    body->ApplyLinearImpulse(b2Vec2(ix, iy), b2Vec2(px, py), wake);
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeApplyLinearImpulseToCenter
    (JNIEnv* env, jclass cls, jlong ptr, jfloat ix, jfloat iy, jboolean wake) {
    b2Body* body = (b2Body*)ptr;
    body->ApplyLinearImpulseToCenter(b2Vec2(ix, iy), wake);
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeApplyAngularImpulse
    (JNIEnv* env, jclass cls, jlong ptr, jfloat impulse, jboolean wake) {
    b2Body* body = (b2Body*)ptr;
    body->ApplyAngularImpulse(impulse, wake);
}

JNIEXPORT jfloat JNICALL Java_org_ironmaple_simulation_physics_Body_nativeGetMass
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    return body->GetMass();
}

JNIEXPORT jfloat JNICALL Java_org_ironmaple_simulation_physics_Body_nativeGetInertia
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    return body->GetInertia();
}

JNIEXPORT jint JNICALL Java_org_ironmaple_simulation_physics_Body_nativeGetType
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    return (jint)body->GetType();
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeSetType
    (JNIEnv* env, jclass cls, jlong ptr, jint type) {
    b2Body* body = (b2Body*)ptr;
    body->SetType((b2BodyType)type);
}

JNIEXPORT jboolean JNICALL Java_org_ironmaple_simulation_physics_Body_nativeIsBullet
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    return body->IsBullet();
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeSetBullet
    (JNIEnv* env, jclass cls, jlong ptr, jboolean bullet) {
    b2Body* body = (b2Body*)ptr;
    body->SetBullet(bullet);
}

JNIEXPORT jboolean JNICALL Java_org_ironmaple_simulation_physics_Body_nativeIsSleepingAllowed
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    return body->IsSleepingAllowed();
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeSetSleepingAllowed
    (JNIEnv* env, jclass cls, jlong ptr, jboolean allowed) {
    b2Body* body = (b2Body*)ptr;
    body->SetSleepingAllowed(allowed);
}

JNIEXPORT jboolean JNICALL Java_org_ironmaple_simulation_physics_Body_nativeIsAwake
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    return body->IsAwake();
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeSetAwake
    (JNIEnv* env, jclass cls, jlong ptr, jboolean awake) {
    b2Body* body = (b2Body*)ptr;
    body->SetAwake(awake);
}

JNIEXPORT jboolean JNICALL Java_org_ironmaple_simulation_physics_Body_nativeIsActive
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    return body->IsEnabled();
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeSetActive
    (JNIEnv* env, jclass cls, jlong ptr, jboolean active) {
    b2Body* body = (b2Body*)ptr;
    body->SetEnabled(active);
}

JNIEXPORT jboolean JNICALL Java_org_ironmaple_simulation_physics_Body_nativeIsFixedRotation
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    return body->IsFixedRotation();
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeSetFixedRotation
    (JNIEnv* env, jclass cls, jlong ptr, jboolean fixed) {
    b2Body* body = (b2Body*)ptr;
    body->SetFixedRotation(fixed);
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeSetLinearDamping
    (JNIEnv* env, jclass cls, jlong ptr, jfloat damping) {
    b2Body* body = (b2Body*)ptr;
    body->SetLinearDamping(damping);
}

JNIEXPORT jfloat JNICALL Java_org_ironmaple_simulation_physics_Body_nativeGetLinearDamping
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    return body->GetLinearDamping();
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeSetAngularDamping
    (JNIEnv* env, jclass cls, jlong ptr, jfloat damping) {
    b2Body* body = (b2Body*)ptr;
    body->SetAngularDamping(damping);
}

JNIEXPORT jfloat JNICALL Java_org_ironmaple_simulation_physics_Body_nativeGetAngularDamping
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    return body->GetAngularDamping();
}

JNIEXPORT jfloat JNICALL Java_org_ironmaple_simulation_physics_Body_nativeGetGravityScale
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Body* body = (b2Body*)ptr;
    return body->GetGravityScale();
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Body_nativeSetGravityScale
    (JNIEnv* env, jclass cls, jlong ptr, jfloat scale) {
    b2Body* body = (b2Body*)ptr;
    body->SetGravityScale(scale);
}

// ============ Fixture Functions ============

JNIEXPORT jboolean JNICALL Java_org_ironmaple_simulation_physics_Fixture_nativeIsSensor
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Fixture* fixture = (b2Fixture*)ptr;
    return fixture->IsSensor();
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Fixture_nativeSetSensor
    (JNIEnv* env, jclass cls, jlong ptr, jboolean sensor) {
    b2Fixture* fixture = (b2Fixture*)ptr;
    fixture->SetSensor(sensor);
}

JNIEXPORT jfloat JNICALL Java_org_ironmaple_simulation_physics_Fixture_nativeGetFriction
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Fixture* fixture = (b2Fixture*)ptr;
    return fixture->GetFriction();
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Fixture_nativeSetFriction
    (JNIEnv* env, jclass cls, jlong ptr, jfloat friction) {
    b2Fixture* fixture = (b2Fixture*)ptr;
    fixture->SetFriction(friction);
}

JNIEXPORT jfloat JNICALL Java_org_ironmaple_simulation_physics_Fixture_nativeGetRestitution
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Fixture* fixture = (b2Fixture*)ptr;
    return fixture->GetRestitution();
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Fixture_nativeSetRestitution
    (JNIEnv* env, jclass cls, jlong ptr, jfloat restitution) {
    b2Fixture* fixture = (b2Fixture*)ptr;
    fixture->SetRestitution(restitution);
}

JNIEXPORT jfloat JNICALL Java_org_ironmaple_simulation_physics_Fixture_nativeGetDensity
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Fixture* fixture = (b2Fixture*)ptr;
    return fixture->GetDensity();
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Fixture_nativeSetDensity
    (JNIEnv* env, jclass cls, jlong ptr, jfloat density) {
    b2Fixture* fixture = (b2Fixture*)ptr;
    fixture->SetDensity(density);
}

// ============ Contact Functions ============

JNIEXPORT jboolean JNICALL Java_org_ironmaple_simulation_physics_Contact_nativeIsTouching
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Contact* contact = (b2Contact*)ptr;
    return contact->IsTouching();
}

JNIEXPORT jboolean JNICALL Java_org_ironmaple_simulation_physics_Contact_nativeIsEnabled
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Contact* contact = (b2Contact*)ptr;
    return contact->IsEnabled();
}

JNIEXPORT void JNICALL Java_org_ironmaple_simulation_physics_Contact_nativeSetEnabled
    (JNIEnv* env, jclass cls, jlong ptr, jboolean enabled) {
    b2Contact* contact = (b2Contact*)ptr;
    contact->SetEnabled(enabled);
}

JNIEXPORT jint JNICALL Java_org_ironmaple_simulation_physics_Contact_nativeGetChildIndexA
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Contact* contact = (b2Contact*)ptr;
    return contact->GetChildIndexA();
}

JNIEXPORT jint JNICALL Java_org_ironmaple_simulation_physics_Contact_nativeGetChildIndexB
    (JNIEnv* env, jclass cls, jlong ptr) {
    b2Contact* contact = (b2Contact*)ptr;
    return contact->GetChildIndexB();
}

} // extern "C"
