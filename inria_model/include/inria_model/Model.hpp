#ifndef INRIA_MODEL_MODEL_HPP
#define INRIA_MODEL_MODEL_HPP

#include <vector>
#include <map>
#include <string>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>

/**
 * Add typedef to Eigen namespace
 * for spatial vector
 */
namespace Eigen {
    typedef Matrix<double, 6, 1> Vector6d;
    typedef Matrix<double, 6, 6> Matrix6d;
}

namespace inria {

/**
 * Shortcut RBDL namespaces
 */
namespace RBDL = RigidBodyDynamics;
namespace RBDLMath = RigidBodyDynamics::Math;

/**
 * Model
 *
 * Wrapper around RBDL API, 
 * URDF limits and degrees 
 * of freedom state.
 * Non thread safe.
 * Always include a 6 DoFs floating base.
 */
class Model
{
    public:

        /**
         * Structure for Link visual data.
         * transform from local frame and
         * mesh filename path.
         * Optional material color.
         */
        struct Visual {
            Eigen::Vector3d transformXYZ;
            Eigen::Vector3d transformRPY;
            Eigen::Vector3d transformScale;
            std::string meshFilename;
            bool isColor;
            Eigen::Vector4d color;
        };
        
        /**
         * Empty initialization
         */
        Model();

        /**
         * Initialization with URDF file.
         *
         * @param filename Path to URDF XML file.
         * @param rootFrameName Frame name (from URDF) 
         * of the root of the kinematics tree. 
         * (Reconstruction of RBDL model tree structure 
         * with given body name as the new root).
         */
        Model(
            const std::string& filename,
            const std::string& rootFrameName = "ROOT");

        /**
         * Initialization with URDF 
         * file or content. 
         * The model must be uninitialized 
         * (default constructor)
         *
         * @param filename Path to URDF XML file.
         * @param fileContent String containing the
         * whole URDF file content to be parsed.
         * @param rootFrameName Frame name (from URDF) 
         * of the root of the kinematics tree. 
         * (Reconstruction of RBDL model tree structure 
         * with given body name as the new root).
         */
        void initFromFile(
            const std::string& filename,
            const std::string& rootFrameName = "ROOT");
        void initFromString(
            const std::string& fileContent,
            const std::string& rootFrameName = "ROOT");

        /**
         * Return typical model size.
         *
         * @return the total number of degrees
         * of freedom including robot's joints and
         * the (6 DoFs) floating base.
         * @return the number of revolute joints.
         * @return the size of RBDL position vector
         * (joints and floating base)
         * @return the size of RBDL velocity and effort
         * vector (joints and floating base)
         */
        size_t sizeDOF() const;
        size_t sizeJoint() const;
        size_t sizeVectPos() const;
        size_t sizeVectVel() const;

        /**
         * Read access to mapping from name to index.
         * 
         * @param Reference to degrees of freedom 
         * mapping from name to index in RBDL position vector.
         * @param Reference to frame
         * mapping from name to RBDL bodies id.
         */
        const std::map<std::string, size_t>& getMappingDOFs() const;
        const std::map<std::string, unsigned int>& getMappingFrames() const;

        /**
         * Read access to links visual data 
         * container indexed by link's name
         */
        const std::map<std::string, Visual>& getMappingVisuals() const;

        /**
         * Retrieve from a degree of freedom 
         * or joint or frame name to its index (or Id).
         *
         * @param name Degree of freedom or RBDL frame name.
         * @return Degree of freedom index in position/velocity 
         * vector or RBDL body Id.
         */
        unsigned int getIndexDOF(const std::string& name) const;
        unsigned int getIndexJoint(const std::string& name) const;
        unsigned int getIndexFrame(const std::string& name) const;

        /**
         * @return the RBDL degrees of freedom or frame 
         * name associated with given index or 
         * throw an std::logic_error if the given 
         * index is invalid.
         */
        const std::string& getNameDOF(size_t index) const;
        const std::string& getNameJoint(size_t index) const;
        const std::string& getNameFrame(size_t index) const;

        /**
         * Get or set a degrees of freedom position
         * from its raw RBDL index or name.
         * Throw std::logic_error in case of invalid
         * index or name.
         *
         * @param index RBDL index in 0:sizeDOF().
         * @param name RBDL floating base or joint name.
         * @param val Degrees of freedom position value.
         */
        double getDOFPos(size_t index) const;
        void setDOFPos(size_t index, double val);
        double getDOFPos(const std::string& name) const;
        void setDOFPos(const std::string& name, double val);

        /**
         * Get or set a degrees of freedom velocity
         * from its raw RBDL index or name.
         * Throw std::logic_error in case of invalid
         * index or name.
         *
         * @param index RBDL index in 0:sizeDOF().
         * @param name RBDL floating base or joint name.
         * @param val Degrees of freedom velocity value.
         */
        double getDOFVel(size_t index) const;
        void setDOFVel(size_t index, double val);
        double getDOFVel(const std::string& name) const;
        void setDOFVel(const std::string& name, double val);

        /**
         * Get or set the sub Eigen::Vector of joint
         * angular positions.
         * Throw std::logic_error in case of invalid
         * vector length.
         */
        Eigen::VectorBlock<const Eigen::VectorXd, -1> 
            getJointPosVect() const;
        void setJointPosVect(const Eigen::VectorXd& vect);
        
        /**
         * Get or set the sub Eigen::Vector of joint
         * angular velocities.
         * Throw std::logic_error in case of invalid
         * vector length.
         */
        Eigen::VectorBlock<const Eigen::VectorXd, -1> 
            getJointVelVect() const;
        void setJointVelVect(const Eigen::VectorXd& vect);

        /**
         * Read only direct access to the complete 
         * position state vector of size sizeVectPos()
         */
        const Eigen::VectorXd& getDOFPosVect() const;

        /**
         * Assign the full degrees of freedom 
         * position vector of size sizeVectPos().
         */
        void setDOFPosVect(const Eigen::VectorXd& vect);

        /**
         * Get or set the floating base position
         * and orientation through quaternion format.
         * Position is the translation in (X,Y,Z) format.
         * Orientation is given in quaternion (X,Y,Z,W) format.
         * Throw std::logic_error in case of uninitialized model.
         */
        Eigen::Vector3d getBasePos() const;
        void setBasePos(const Eigen::Vector3d& pos);
        Eigen::Quaterniond getBaseQuat() const;
        void setBaseQuat(const Eigen::Quaterniond& quat);

        /**
         * Get or set the floating base linear
         * and angular velocity vector.
         * WARNING:
         * The linear velocity of the floating base 
         * with respect to world origin is expressed
         * in world frame.
         * The angular velocity of the floating base 
         * with respect to world origin is expressed
         * in the base LOCAL BODY FRAME.
         * Throw std::logic_error in case of uninitialized model.
         */
        Eigen::Vector3d getBaseVelLinear() const;
        void setBaseVelLinear(const Eigen::Vector3d& vel);
        Eigen::Vector3d getBaseVelAngular() const;
        void setBaseVelAngular(const Eigen::Vector3d& vel);

        /**
         * Read only direct access to the complete
         * velocity state vector of size sizeVectVel()
         */
        const Eigen::VectorXd& getDOFVelVect() const;

        /**
         * Assign the full degrees of freedom
         * velocity vector of size sizeVectVel().
         */
        void setDOFVelVect(const Eigen::VectorXd& vect);

        /**
         * Import and copy to this all degrees of 
         * freedom state from given other model. 
         * The copy is based on names so actual RBDL 
         * tree root (indexes) can be different.
         *
         * @param model Model whose state will be copied.
         * The model's state must be updated.
         * The model will not be changed (const).
         */
        void importState(Model& model);

        /**
         * @return true if the model's degrees 
         * of freedom are marked as updated and 
         * RBDL is out of sync.
         */
        bool isDirty() const;

        /**
         * Update the underlying RBDL model with all 
         * current degrees of freedom position 
         * and velocity values
         */
        void updateState();

        /**
         * Read only access to joint limits.
         * Lower and upper position angle limit, maximum
         * velocity and torque.
         *
         * @return a view block vector of size sizeJoint().
         */
        Eigen::VectorBlock<const Eigen::VectorXd, -1>
            jointLimitsLower() const;
        Eigen::VectorBlock<const Eigen::VectorXd, -1>
            jointLimitsUpper() const;
        Eigen::VectorBlock<const Eigen::VectorXd, -1>
            jointLimitsVelocity() const;
        Eigen::VectorBlock<const Eigen::VectorXd, -1>
            jointLimitsTorque() const;

        /**
         * Compute the given 3d point position expressed
         * in srcFrame and return the result
         * with respect to dstFrame.
         * Internal degrees of freedom positions are used.
         * WARNING: to be correct, internal state must have
         * been updated using positions.
         * Frames are given either by their names, or by
         * their RBDL body ids.
         * Throw std::logic_error in case of invalid names.
         *
         * @param point Optional 3d point given in srcFrame.
         * (default is zero translation).
         * @return computed point in dstFrame.
         */
        Eigen::Vector3d position(
            size_t srcFrameId, 
            size_t dstFrameId,
            const Eigen::Vector3d& point = Eigen::Vector3d::Zero()) const;
        Eigen::Vector3d position(
            const std::string& srcFrameName, 
            const std::string& dstFrameName,
            const Eigen::Vector3d& point = Eigen::Vector3d::Zero()) const;

        /**
         * Compute the orientation of a source frame
         * expressed in a destination frame.
         * Internal degrees of freedom positions are used.
         * WARNING: to be correct, internal state must have
         * been updated using positions.
         * Frames are given either by their names, or by
         * their RBDL body ids.
         * Throw std::logic_error in case of invalid names.
         *
         * @return computed rotation matrix from srcFrame to dstFrame.
         * Unit vectors of srcFrame are expressed with respect 
         * to dstFrame.
         */
        Eigen::Matrix3d orientation(
            size_t srcFrameId, 
            size_t dstFrameId) const;
        Eigen::Matrix3d orientation(
            const std::string& srcFrameName, 
            const std::string& dstFrameName) const;
        
        /**
         * Compute the Euler Yaw orientation
         * of a source frame expressed in 
         * destination frame.
         * Internal degrees of freedom positions are used.
         * WARNING: to be correct, internal state must have
         * been updated using positions.
         * Frames are given either by their names, or by
         * their RBDL body ids.
         *
         * @return the Euler yaw angle in -PI:PI
         * of the rotation of srcFrame around
         * the Z axis of dstFrame.
         */
        double orientationYaw(
            size_t srcFrameId, 
            size_t dstFrameId) const;
        double orientationYaw(
            const std::string& srcFrameName, 
            const std::string& dstFrameName) const;

        /**
         * Create and return the spatial transform matrix
         * converting a wrench expressed in src frame 
         * to dst frame.
         *
         * @param pos The position of the center of 
         * the dst frame expressed in src frame.
         * @param mat The orientation of the of 
         * the dst frame expressed in src frame.
         *
         * @return the spatial wrench transform matrix (6x6).
         */
        Eigen::Matrix6d getWrenchTransform(
            const Eigen::Vector3d& posDstInSrc,
            const Eigen::Matrix3d& matDstInSrc) const;

        /**
         * Transform and return a torque-force 6D 
         * wrench from srcFrame to dstFrame.
         * Internal degrees of freedom positions are used.
         * WARNING: to be correct, internal state must have
         * been updated using positions.
         * Frames are given either by their names, or by
         * their RBDL body ids.
         * Throw std::logic_error in case of invalid names.
         *
         * @param wrench Input wrench expressed in 
         * srcFrame to be transformed.
         * @return computed wrench in dstFrame.
         */
        Eigen::Vector6d wrenchTransform(
            size_t srcFrameId, 
            size_t dstFrameId,
            const Eigen::Vector6d& wrenchInSrc) const;
        Eigen::Vector6d wrenchTransform(
            const std::string& srcFrameName, 
            const std::string& dstFrameName,
            const Eigen::Vector6d& wrenchInSrc) const;

        /**
         * Update the floating base position such 
         * that given frame name position matches 
         * assigned position in world frame.
         *
         * @param frame Frame id or name to be placed at
         * specific pose in world frame.
         * @param posInWorld Expected position of given
         * frame center expressed in world frame.
         */
        void setBaseToMatchFramePosition(
            size_t frameId, 
            const Eigen::Vector3d& posInWorld);
        void setBaseToMatchFramePosition(
            const std::string& frameName, 
            const Eigen::Vector3d& posInWorld);

        /**
         * Update the floating base orientation such 
         * that given frame name orientation matches 
         * assigned quaternion in world frame.
         *
         * @param frame Frame id or name to be placed at
         * specific pose in world frame.
         * @param quatInWorld Expected orientation
         * (in quaternion format) of point in frameName frame
         * expressed in world frame.
         */
        void setBaseToMatchFrameOrientation(
            size_t frameId, 
            const Eigen::Quaterniond& quatInWorld);
        void setBaseToMatchFrameOrientation(
            const std::string& frameName, 
            const Eigen::Quaterniond& quatInWorld);

        /**
         * Update the floating base pose such that
         * given frame name pose match assigned pose 
         * in world frame.
         *
         * @param frame Frame id or name to be placed at
         * specific pose in world frame.
         * @param pointInFrame Relative position of 
         * constrained point expressed in frameName frame.
         * @param posInWorld Expected position of point
         * in frameName frame expressed in world frame.
         * @param quatInWorld Expected orientation
         * (in quaternion format) of point in frameName frame
         * expressed in world frame.
         */
        void setBaseToMatchFramePose(
            size_t frameId, 
            const Eigen::Vector3d& pointInFrame,
            const Eigen::Vector3d& posInWorld,
            const Eigen::Quaterniond& quatInWorld);
        void setBaseToMatchFramePose(
            const std::string& frameName, 
            const Eigen::Vector3d& pointInFrame,
            const Eigen::Vector3d& posInWorld,
            const Eigen::Quaterniond& quatInWorld);

        /**
         * Update the floating base velocity such
         * that given frame name velocity matches
         * given velocity vector expressed in world.
         *
         * @param frame Frame id or name whose velocity to be 
         * fixed by updating the floating base velocity.
         * @param velocityInWorld Velocity of be assign to frame
         * expressed in world frame.
         */
        void setBaseVeltoMatchFrameVel(
            size_t frameId, 
            const Eigen::Vector6d& velocityInWorld);
        void setBaseVeltoMatchFrameVel(
            const std::string& frameName, 
            const Eigen::Vector6d& velocityInWorld);
        
        /**
         * Compute the center of mass position.
         * Internal degrees of freedom positions are used.
         * WARNING: to be correct, internal state must have
         * been updated using positions.
         * Destination frames is given either by its name, 
         * or by its RBDL body id.
         *
         * @return the position of CoM expressed 
         * in dstFrame.
         */
        Eigen::Vector3d centerOfMass(
            size_t dstFrameId) const;
        Eigen::Vector3d centerOfMass(
            const std::string& dstFrameName) const;

        /**
         * @return the total mass of the model
         */
        double massSum() const;

        /**
         * Assign the gravity force.
         *
         * @param vect The gravity vector in world frame.
         */
        void setGravity(const Eigen::Vector3d& vect);

        /**
         * Compute 6D Cartesian velocity of 
         * a point on kinematics tree.
         * Internal degrees of freedom positions and
         * velocities are used.
         * WARNING: to be correct, internal state must have
         * been updated using positions.
         *
         * @param frameSrcId, frameSrcName RBDL frame id or name
         * of point whose velocity is computed.
         * @param frameDstId, frameDstName RBDL frame id or 
         * name in which the computed velocity is expressed.
         * Note: Additional computing is needed if not world frame.
         * WARNING: To be correct, destination frame must be fixed 
         * with respect to world frame.
         * @param point Relative position of 
         * asked point expressed in frameName frame.
         * @return the 6D (rotation, translation) velocity 
         * at point in frameSrc expressed in frameDst.
         */
        Eigen::Vector6d pointVelocity(
            size_t frameSrcId,
            size_t frameDstId,
            const Eigen::Vector3d& point = Eigen::Vector3d::Zero());
        Eigen::Vector6d pointVelocity(
            const std::string& frameSrcName, 
            const std::string& frameDstName, 
            const Eigen::Vector3d& point = Eigen::Vector3d::Zero());
        
        /**
         * Compute 6D Cartesian acceleration of 
         * a point on kinematics tree.
         * Internal degrees of freedom positions 
         * and velocities are used.
         *
         * @param frameSrcId, frameSrcName RBDL frame id or name
         * of point whose acceleration is computed.
         * @param frameDstId, frameDstName RBDL frame id or 
         * name in which the computed acceleration is expressed.
         * Note: Additional computing is needed if not world frame.
         * WARNING: To be correct, destination frame must be fixed 
         * with respect to world frame.
         * @param acceleration Used degrees of freedom
         * acceleration (sizeVectVel).
         * @param point Relative position of 
         * asked point expressed in frameName frame.
         * @return the 6D (rotation, translation) acceleration
         * at point in frameSrc expressed in frameDst.
         */
        Eigen::Vector6d pointAcceleration(
            size_t frameSrcId,
            size_t frameDstId,
            const Eigen::VectorXd& acceleration,
            const Eigen::Vector3d& point = Eigen::Vector3d::Zero());
        Eigen::Vector6d pointAcceleration(
            const std::string& frameSrcName, 
            const std::string& frameDstName, 
            const Eigen::VectorXd& acceleration,
            const Eigen::Vector3d& point = Eigen::Vector3d::Zero());

        /**
         * Compute and return the Jacobian matrix
         * at a given point and body frame.
         * Internal degrees of freedom positions are used.
         * WARNING: to be correct, internal state must have
         * been updated using positions.
         *
         * @param frameSrcId, frameSrcName The frame by its 
         * RBDL body id or name in which the point is expressed.
         * @param frameDstId, frameDstName The frame by its 
         * RBDL body id or name in which the Jacobian is expressed.
         * Note: Additional computing is needed if not world frame.
         * @param point 3d point expressed in frameSrc
         * where the Jacobian is computed. Zero by default.
         * @return the 6 x sizeDOF() Jacobian matrix
         * in spatial vector format (rotation and translation)
         * (axisX, axisY, axisZ, transX, transY, transZ)
         * expressed in frameDst frame.
         */
        Eigen::MatrixXd pointJacobian(
            size_t frameSrcId,
            size_t frameDstId,
            const Eigen::Vector3d& point = Eigen::Vector3d::Zero()) const;
        Eigen::MatrixXd pointJacobian(
            const std::string& frameSrcName,
            const std::string& frameDstName,
            const Eigen::Vector3d& point = Eigen::Vector3d::Zero()) const;

        /**
         * Compute and return the Jacobian matrix 
         * of robot's center of mass.
         * Internal degrees of freedom positions are used.
         * WARNING: to be correct, internal state must have
         * been updated using positions.
         *
         * @param frameDstId, frameDstName The frame by its 
         * RBDL body id or name in which the Jacobian is expressed.
         * Note: Additional computing is needed if not world frame.
         * @return the 3 x sizeDOF() Jacobian matrix 
         * (transX, transY, transZ) expressed in frameDst.
         */
        Eigen::MatrixXd comJacobian(
            size_t frameDstId) const;
        Eigen::MatrixXd comJacobian(
            const std::string& frameSrcName) const;

        /**
         * Compute the joint space gravity force vector
         * Internal degrees of freedom positions 
         * are used.
         * WARNING: to be correct, internal state must have
         * been updated using positions.
         *
         * @return the joint space gravity 
         * vector (sizeDOF x 1)
         */
        Eigen::VectorXd computeGravityVector();

        /**
         * Compute equation of motion joint-space inertia 
         * matrix and joint-space bias force vector 
         * (gravity, centrifugal and Coriolis).
         * Optionally also compute the gravity 
         * vector term alone.
         * Internal degrees of freedom positions 
         * and velocities are used.
         * WARNING: to be correct, internal state must have
         * been updated using positions and velocities.
         *
         * @param H (output) Assigned joint-space inertia
         * matrix (sizeVectVel() x sizeVectVel())
         * @param C (output) Assigned joint-space bias force
         * vector (sizeVectVel()).
         * @param G (optional output) Assigned gravity force
         * vector (sizeVectVel()).
         */
        void computeEquationOfMotion(
            Eigen::MatrixXd& H,
            Eigen::VectorXd& C,
            Eigen::VectorXd* G = nullptr);

        /**
         * Compute the joint torques assuming that the model is
         * attached to the world not though its floating base 
         * but instead though given frame link by a 6d contact.
         *
         * Internal degrees of freedom positions 
         * are used.
         * WARNING: to be correct, internal state must have
         * been updated using positions.
         *
         * @param frame Frame id or name assume to be a 6d contact
         * through which the model is attached to the world.
         * @param wrench Optional computed contact wrench.
         * @return the computed joint torque (sizeJoint()).
         */
        Eigen::VectorXd computeJointTorqueSingleContact(
            size_t frameId,
            Eigen::Vector6d* wrench = nullptr);
        Eigen::VectorXd computeJointTorqueSingleContact(
            const std::string& frameName,
            Eigen::Vector6d* wrench = nullptr);

        /**
         * Compute the centroidal momentum matrix 
         * (CMM) as well as the bias vector 
         * CMMDot.QDot from centroidal dynamics equations.
         * Internal degrees of freedom positions 
         * and velocities are used.
         * WARNING: to be correct, internal state must have
         * been updated using positions.
         * CMM matrix is such that:
         * h = CMM*QDot 
         * where h is the 6D (angular, linear) momentum of 
         * the system center of mass expressed in world frame.
         * Bias vector is such that:
         * hDot = CMM*QDDot + CMMDot*QDot
         *
         * @param H Joint-space inertia matrix 
         * (sizeVectVel() x sizeVectVel())
         * @param C Joint-space bias force
         * vector (gravity, centrifugal and Coriolis) 
         * (sizeVectVel()).
         * @param G Only Gravity force vector (sizeVectVel()).
         * @param CMM (output) Assigned centroidal 
         * momentum matrix (6 x sizeVectVel()).
         * @param CMMDotQDot (output) Assigned bias vector
         * (derivative of CMM matrix multiplied with velocity 
         * vector) (size 6).
         */
        void computeCentroidalDynamics(
            const Eigen::MatrixXd& H,
            const Eigen::VectorXd& C,
            const Eigen::VectorXd& G,
            Eigen::MatrixXd& CMM,
            Eigen::VectorXd& CMMDotQDot);

        /**
         * Compute classical Inverse Dynamics (recursive Newton-Euler) 
         * on tree model and return computed torques for each 
         * degrees of freedom (including floating base) using 
         * current position. 
         * Internal degrees of freedom positions and
         * velocities are used.
         * WARNING: to be correct, internal state must have
         * been updated using positions.
         *
         * @param acceleration Degrees of freedom 
         * acceleration vector (sizeVectVel()).
         * @param externalWrenches Mapping from frame name
         * to external (torque, force) wrenches applied on 
         * the associated link expressed in local body frame.
         * @return degrees of freedom torque vector (sizeVectVel()).
         */
        Eigen::VectorXd inverseDynamics(
            const Eigen::VectorXd& acceleration,
            const std::map<std::string, Eigen::Vector6d>& 
                externalWrenches = {});

        /**
         * Compute Inverse Dynamics on a modified closed loop
         * model where given frame id is considered fixed
         * in base coordinates. Computed torques are returned
         * and floating base degrees of freedom are also computed.
         * Since closed loop system have infinite solution for
         * Inverse Dynamics, the solution minimizing the norm 2
         * of joint torques is chosen.
         * Current positions and velocities are used.
         * WARNING: to be correct, internal state must have
         * been updated using positions.
         *
         * @param frameFixedId, frameFixedName Frame id 
         * or name assumed to be fixed in world frame.
         * @param acceleration Degrees of freedom 
         * acceleration vector (sizeVectVel()).
         * @param contactWrench If not null, the cartesian 6D 
         * moment and linear forces (Nx, Ny, Nz, Fx, Fy, Fz) 
         * expressed in fixed body frame are assigned.
         * @return degrees of freedom torque vector (sizeVectVel()).
         */
        Eigen::VectorXd inverseDynamicsContact(
            size_t frameFixedId,
            const Eigen::VectorXd& acceleration,
            Eigen::Vector6d* contactWrench = nullptr);
        Eigen::VectorXd inverseDynamicsContact(
            const std::string& frameFixedName,
            const Eigen::VectorXd& acceleration,
            Eigen::Vector6d* contactWrench = nullptr);
        
        /**
         * Compute direct forward dynamics on Model 
         * with given contact constraints set.
         * Directly solve the system for acceleration 
         * and Cartesian contact force:
         * |H G'|.|ddq    | = |tau-C|
         * |G 0 | |-lambda|   |gamma|
         * WARNING: Internal RBDL model position or velocity 
         * state are updated. A call to updateState()
         * is required.
         *
         * @param constraints RBDL constraints set.
         * @param position Used degrees of freedom position 
         * vector including floating base quaternion (sizeVectPos()).
         * @param velocity Used degrees of freedom velocity
         * vector (sizeVectVel()).
         * @param torque Used degrees of freedom torque 
         * vector (sizeVectVel()).
         * @return acceleration vector of size sizeVectVel().
         */
        Eigen::VectorXd forwardDynamicsContacts(
            RBDL::ConstraintSet& constraints, 
            const Eigen::VectorXd& position,
            const Eigen::VectorXd& velocity,
            const Eigen::VectorXd& torque);

        /**
         * Compute forward impulsive dynamics on Model 
         * with given contact constraints set.
         * Solve the system for velocity and Cartesian
         * contact force:
         * |H dt.G'|.|dq+    | = |dt.(tau-C)+H.dq-|
         * |G 0    | |-lambda|   |0               |
         * WARNING: Internal RBDL model position or velocity 
         * state are updated. A call to updateState()
         * is required.
         *
         * @param dt Integration time step in seconds.
         * @param constraints RBDL constraints set.
         * @param position Used degrees of freedom position 
         * vector including floating base quaternion (sizeVectPos()).
         * @param velocity Used degrees of freedom velocity
         * vector (sizeVectVel()).
         * @param torque Used degrees of freedom torque 
         * vector (sizeVectVel()).
         * @return new velocity vector of size sizeVectVel().
         */
        Eigen::VectorXd forwardImpulsiveDynamicsContacts(
            double dt, 
            RBDL::ConstraintSet& constraints, 
            const Eigen::VectorXd& position,
            const Eigen::VectorXd& velocity,
            const Eigen::VectorXd& torque);

        /**
         * Compute the collision velocity impulses
         * for given ConstraintSet. The new computed
         * velocities accounting for the collision
         * are returned. Current position and old
         * velocity are given.
         * Solve the system for velocity and Cartesian
         * contact impulses:
         * |H G'|.|dq+   | = |H.dq-|
         * |G 0 | |Lambda|   |0    |
         * WARNING: Internal RBDL model position or velocity 
         * state are updated. A call to updateState()
         * is required.
         *
         * @param constraints RBDL constraints set.
         * @param position Used degrees of freedom position 
         * vector including floating base quaternion (sizeVectPos()).
         * @param velocity Used degrees of freedom velocity
         * vector (sizeVectVel()).
         * @return new velocity vector of size sizeVectVel().
         */
        Eigen::VectorXd impulseContacts(
            RBDL::ConstraintSet& constraints,
            const Eigen::VectorXd& position,
            const Eigen::VectorXd& velocity);
        
        /**
         * Retrieve a body model mass and center of mass.
         *
         * @param frameId, frameName The link given by 
         * its RBDL body id or name.
         * @return body mass or center of mass expressed
         * in local body frame.
         */
        double getBodyMass(size_t frameId) const;
        double getBodyMass(const std::string& frameName) const;
        Eigen::Vector3d getBodyCoM(size_t frameId) const;
        Eigen::Vector3d getBodyCoM(const std::string& frameName) const;

        /**
         * Compute potential and kinetic energy.
         * Internal degrees of freedom positions 
         * and velocities are used.
         * WARNING: to be correct, internal state must have
         * been updated using positions.
         *
         * @return total potential or kinetic 
         * energy computed in world frame.
         */
        double energyPotential() const;
        double energyKinetic() const;

        /**
         * Direct access to RBDL model
         */
        const RBDL::Model& getRBDLModel() const;
        RBDL::Model& getRBDLModel();

        /**
         * Return path to loaded URDf file
         */
        const std::string& getPathURDF() const;

    private:
        
        /**
         * RBDL model instance
         */
        RBDL::Model _model;

        /**
         * Path to loaded URDF modelfile
         */
        std::string _pathURDF;

        /**
         * Frame id and name for floating base root
         */
        size_t _baseFrameId;
        std::string _baseFrameName;

        /**
         * True if the internal position or velocity 
         * have been assigned but the internal RBDL model 
         * kinematics is not yet updated.
         * Used as safe guard against user error.
         */
        bool _isDirty;

        /**
         * Current DOFs position with 
         * following RBDL structure:
         * - Floating base 3d position vector expressed 
         * in world frame (t_x, t_y, t_z) (in meters).
         * - Floating base orientation quaternion
         * 3d imaginary part (quat_x, quat_y, quat_z)
         * with respect to world frame (in radian).
         * - All joints angular position (in radians) (sizeJoint()).
         * - Last element is the floating base orientation
         * quaternion scalar real part (quat_w).
         * Total size is #DoF+1 (sizeVectPos()).
         */
        Eigen::VectorXd _position;

        /**
         * Current DOFs velocity with 
         * following RBDL structure:
         * - Floating base 3d linear velocity vector
         * (motion of base with respect to origin) expressed
         * in world frame (in meters per second).
         * - Floating base 3d angular velocity vector
         * (motion of base with respect to origin) EXPRESSED
         * In FLOATING BASE BODY frame (in radians per second).
         * - All joints angular velocity (in radians per second).
         * Total size is #DoF (sizeVectVel()).
         */
        Eigen::VectorXd _velocity;

        /**
         * Joint limits from URDF file.
         * For each joint, the vectors hold
         * the lower and upper valid angular range,
         * absolute maximum velocity and command effort.
         * Total size is #DoF+1 but only
         * joint indexes in 6:#DoF are used.
         */
        Eigen::VectorXd _limitPosLower;
        Eigen::VectorXd _limitPosUpper;
        Eigen::VectorXd _limitVelocity;
        Eigen::VectorXd _limitEffort;
        
        /**
         * Joint index to name 
         * and name to index mapping.
         * Indexes are associated to index in
         * joint position and limit vectors.
         */
        std::vector<std::string> _dofIndexToName;
        std::map<std::string, size_t> _dofNameToIndex;

        /**
         * Container of links visual data 
         * associated with link's name
         */
        std::map<std::string, Visual> _visuals;
};

}

#endif

