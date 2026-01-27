package org.ironmaple.simulation.gamepieces;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.utils.mathutils.GeometryConvertor;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;

/**
 *
 *
 * <h1>Simulates a Game Piece on the Field.</h1>
 *
 * <p>This class simulates a game piece on the field, which has a collision space and interacts with other objects.
 *
 * <p>Game pieces can be "grabbed" by an {@link IntakeSimulation}.
 *
 * <p>For the simulation to actually run, every instance must be added to a
 * {@link org.ironmaple.simulation.SimulatedArena} through
 * {@link SimulatedArena#addGamePiece(GamePieceOnFieldSimulation)}.
 */
public class GamePieceOnFieldSimulation implements GamePiece {
    public static final double COEFFICIENT_OF_FRICTION = 0.8, MINIMUM_BOUNCING_VELOCITY = 0.2;

    /**
     *
     *
     * <h2>Supplier of the Current Z-Pose (Height) of the Game Piece.</h2>
     *
     * <p>Normally, the height is fixed at half the thickness of the game piece to simulate it being "on the ground."
     *
     * <p>If the game piece is flying at a low height, the height is calculated using the law of free-fall.
     */
    private final DoubleSupplier zPositionSupplier;
    /**
     *
     *
     * <h2>The Type of the Game Piece.</h2>
     *
     * <p>Affects the result of {@link SimulatedArena#getGamePiecesPosesByType(String)}.
     */
    public final String type;

    /** The Box2D body for this game piece */
    protected Body body;

    /** The main fixture for collision */
    protected Fixture fixture;

    /** Info about this game piece type */
    protected final GamePieceInfo info;

    /** Initial pose */
    private final Pose2d initialPose;

    /** Initial velocity */
    private final Translation2d initialVelocity;

    /**
     *
     *
     * <h2>Creates a Game Piece on the Field with Fixed Height.</h2>
     *
     * @param info info about the game piece type
     * @param initialPose the initial position of the game piece on the field
     */
    public GamePieceOnFieldSimulation(GamePieceInfo info, Pose2d initialPose) {
        this(info, () -> info.gamePieceHeight.in(Meters) / 2, initialPose, new Translation2d());
    }

    /**
     *
     *
     * <h2>Creates a Game Piece on the Field with Custom Height Supplier and Initial Velocity.</h2>
     *
     * @param info info about the game piece type
     * @param zPositionSupplier a supplier that provides the current Z-height of the game piece
     * @param initialPose the initial position of the game piece on the field
     * @param initialVelocityMPS the initial velocity of the game piece, in meters per second
     */
    public GamePieceOnFieldSimulation(
            GamePieceInfo info, DoubleSupplier zPositionSupplier, Pose2d initialPose, Translation2d initialVelocityMPS) {
        this.type = info.type;
        this.zPositionSupplier = zPositionSupplier;
        this.info = info;
        this.initialPose = initialPose;
        this.initialVelocity = initialVelocityMPS;
    }

    /**
     *
     *
     * <h2>Adds this game piece to the physics world.</h2>
     *
     * @param world the Box2D world
     */
    public void addToWorld(World world) {
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set((float) initialPose.getX(), (float) initialPose.getY());
        bd.angle = (float) initialPose.getRotation().getRadians();
        bd.linearDamping = (float) info.linearDamping;
        bd.angularDamping = (float) info.angularDamping;
        bd.bullet = true;

        this.body = world.createBody(bd);

        FixtureDef fd = new FixtureDef();
        fd.shape = info.shape;
        fd.friction = (float) COEFFICIENT_OF_FRICTION;
        fd.restitution = (float) info.coefficientOfRestitution;
        fd.density = (float) (info.gamePieceMass.in(Kilogram) / getShapeArea(info.shape));

        this.fixture = body.createFixture(fd);

        // Set initial velocity
        body.setLinearVelocity(GeometryConvertor.toBox2dVec2(initialVelocity));

        // Store reference to this game piece in the body's user data
        body.setUserData(this);
    }

    /**
     *
     *
     * <h2>Removes this game piece from the physics world.</h2>
     *
     * @param world the Box2D world
     */
    public void removeFromWorld(World world) {
        if (body != null) {
            world.destroyBody(body);
            body = null;
        }
    }

    /**
     *
     *
     * <h2>Gets the approximate area of a shape.</h2>
     */
    private double getShapeArea(Shape shape) {
        // Approximate area calculation
        // For now, use a default value; specific shapes should override
        return 0.01; // Default 100 cm^2
    }

    /**
     *
     *
     * <h2>Gets the Box2D body.</h2>
     *
     * @return the Box2D body
     */
    public Body getBody() {
        return body;
    }

    /**
     *
     *
     * <h2>Sets the world velocity of this game piece.</h2>
     *
     * @param chassisSpeedsWorldFrame the speeds of the game piece
     */
    public void setVelocity(ChassisSpeeds chassisSpeedsWorldFrame) {
        if (body != null) {
            body.setLinearVelocity(GeometryConvertor.toBox2dLinearVelocity(chassisSpeedsWorldFrame));
            body.setAngularVelocity((float) chassisSpeedsWorldFrame.omegaRadiansPerSecond);
        }
    }

    /**
     *
     *
     * <h2>Sets the linear velocity of this game piece.</h2>
     *
     * @param velocity the velocity
     */
    public void setLinearVelocity(Vec2 velocity) {
        if (body != null) {
            body.setLinearVelocity(velocity);
        }
    }

    /**
     *
     *
     * <h2>Gets the linear velocity of this game piece.</h2>
     *
     * @return the linear velocity
     */
    public Vec2 getLinearVelocity() {
        if (body != null) {
            return body.getLinearVelocity();
        }
        return new Vec2(0, 0);
    }

    /**
     *
     *
     * <h2>Obtains the 2d position of the game piece</h2>
     *
     * @return the 2d position of the game piece
     */
    public Pose2d getPoseOnField() {
        if (body != null) {
            return GeometryConvertor.toWpilibPose2d(body.getTransform());
        }
        return initialPose;
    }

    /**
     *
     *
     * <h2>Obtains a 3d pose of the game piece.</h2>
     *
     * <p>The 3d position is calculated from both the {@link #getPoseOnField()} and {@link #zPositionSupplier}
     *
     * @return the 3d position of the game piece
     */
    @Override
    public Pose3d getPose3d() {
        final Pose2d pose2d = getPoseOnField();
        return new Pose3d(
                pose2d.getX(),
                pose2d.getY(),
                zPositionSupplier.getAsDouble(),
                new Rotation3d(0, 0, pose2d.getRotation().getRadians()));
    }

    /**
     *
     *
     * <h2>Stores the info of a type of game piece</h2>
     *
     * @param type the type of the game piece, affecting categorization within the arena
     * @param shape the Box2D shape of the collision space for the game piece
     * @param gamePieceHeight the height (thickness) of the game piece, in meters
     * @param gamePieceMass the mass of the game piece, in kilograms
     */
    public record GamePieceInfo(
            String type,
            Shape shape,
            Distance gamePieceHeight,
            Mass gamePieceMass,
            double linearDamping,
            double angularDamping,
            double coefficientOfRestitution) {}

    public void onIntake(String intakeTargetGamePieceType) {}

    @Override
    public String getType() {
        return this.type;
    }

    @Override
    public Translation3d getVelocity3dMPS() {
        return new Translation3d(GeometryConvertor.toWpilibTranslation2d(this.getLinearVelocity()));
    }

    @Override
    public boolean isGrounded() {
        return true;
    }

    @Override
    public void triggerHitTargetCallBack() {
        return;
    }
}
