#ifndef INRIA_MODEL_PINOCCHIOINTERFACE_HPP
#define INRIA_MODEL_PINOCCHIOINTERFACE_HPP

#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>
#include <inria_model/Model.hpp>

//Include Pinocchio minimum headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#pragma GCC diagnostic pop

namespace inria {

/**
 * PinocchioInterface
 *
 * Use Pinocchio library to efficiently compute
 * model related kinematics or dynamics quantities
 * but expressed using the same convention as RBDL.
 */
class PinocchioInterface
{
    public:

        /**
         * Empty initialization
         */
        PinocchioInterface();

        /**
         * Initialization with URDF file and
         * associated RBDL model wrapper.
         * The 6DOF floating base is added to
         * the default body link.
         *
         * @param modelRBDL Instance wrapper around RBDL model.
         */
        PinocchioInterface(
            const Model& modelRBDL);

        /**
         * Update model kinematics with
         * given position vector.
         *
         * @param posRBDL Position vector in RBDL format.
         * @param velRBDL Velocity vector in RBDL format.
         */
        void updateKinematics(
            const Eigen::VectorXd& posRBDL,
            const Eigen::VectorXd& velRBDL = Eigen::VectorXd());

        /**
         * Compute the given 3d point position expressed
         * in srcFrame and return the result with respect 
         * to dstFrame.
         * Internal degrees of freedom positions are used.
         * WARNING: to be correct, internal state must have
         * been updated using updateKinematics().
         * Frames are given either by their names, or by
         * their RBDL body ids.
         * Throw std::logic_error in case of invalid names.
         *
         * @param point Optional 3d point given in srcFrame.
         * (default is zero translation).
         * @return computed point in dstFrame.
         */
        Eigen::Vector3d position(
            const std::string& srcFrameName, 
            const std::string& dstFrameName,
            const Eigen::Vector3d& point = Eigen::Vector3d::Zero());

        /**
         * Compute the orientation of a source frame
         * expressed in a destination frame.
         * Internal degrees of freedom positions are used.
         * WARNING: to be correct, internal state must have
         * been updated using updateKinematics().
         * Frames are given either by their names, or by
         * their RBDL body ids.
         * Throw std::logic_error in case of invalid names.
         *
         * @return computed rotation matrix from srcFrame to dstFrame.
         * Unit vectors of srcFrame are expressed with respect 
         * to dstFrame.
         */
        Eigen::Matrix3d orientation(
            const std::string& srcFrameName, 
            const std::string& dstFrameName);

        /**
         * Compute and return the Jacobian matrix
         * of a given body frame.
         * Internal degrees of freedom positions are used.
         * WARNING: to be correct, internal state must have
         * been updated using updateKinematics().
         *
         * @param frameSrcName The frame by its 
         * RBDL body name in which the point is expressed.
         * @param frameDstName The frame by its 
         * RBDL body name in which the Jacobian is expressed.
         * WARNING: To be correct, destination frame must be fixed 
         * with respect to world frame.
         * @return the 6 x sizeDOF() Jacobian matrix
         * in spatial vector format (rotation and translation)
         * (axisX, axisY, axisZ, transX, transY, transZ)
         * expressed in frameDst frame.
         */
        Eigen::MatrixXd frameJacobian(
            const std::string& srcFrameName,
            const std::string& dstFrameName);

        /**
         * Compute the joint space gravity force vector
         * Internal degrees of freedom positions 
         * are used.
         * WARNING: to be correct, internal state must have
         * been updated using updateKinematics().
         *
         * @return the joint space gravity 
         * vector (sizeDOF x 1)
         */
        Eigen::VectorXd gravityVector();

        /**
         * Compute the joint space inertia mass matrix.
         * Internal degrees of freedom positions 
         * are used.
         * WARNING: to be correct, internal state must have
         * been updated using updateKinematics().
         *
         * @return the joint space mass matrix 
         * (sizeDOF x sizeDOF)
         */
        Eigen::MatrixXd massMatrix();

        /**
         * Compute the analytical joint space gravity force 
         * vector partial derivatives.
         * Internal degrees of freedom positions 
         * are used.
         * WARNING: to be correct, internal state must have
         * been updated using updateKinematics().
         *
         * @return the joint space gravity 
         * partial derivatives matrix (sizeDOF x sizeDOF)
         */
        Eigen::MatrixXd diffGravityVector();

        /**
         * Compute analytically the Jacobian-wrench product
         * partial derivatives from RNEA differentiated algorithm.
         * The differentiated transposed product 
         * d(J(q)^T.lambda)/dq is computed.
         * The Jacobian and the wrench are expressed 
         * in the local body frame.
         * Internal degrees of freedom positions 
         * are used.
         * WARNING: to be correct, internal state must have
         * been updated using updateKinematics().
         *
         * @param frameName The frame name by its RBDL name
         * where the Jacobian is computed and the wrench expressed.
         * @param wrenchRBDLInLocal Contact wrench in RBDL format
         * and expressed in local contact frame to be
         * multiplied with the Hessian.
         * @return the Hessian-wrench product H^T.w 
         * (sizeDOF x sizeDOF).
         */
        Eigen::MatrixXd diffHessianWrenchInLocalProduct(
            const std::string& frameName,
            const Eigen::Vector6d& wrenchRBDLInLocal);
        
        /**
         * Compute analytically the Jacobian-force product
         * partial derivatives from RNEA differentiated algorithm.
         * The differentiated transposed product 
         * d(J(q)^T.force)/dq is computed.
         * The Jacobian and the force are expressed 
         * in the world frame.
         * Internal degrees of freedom positions 
         * are used.
         * WARNING: to be correct, internal state must have
         * been updated using updateKinematics().
         *
         * @param frameName The frame name by its RBDL name
         * where the Jacobian is computed.
         * @param forceRBDLInWorld Contact linear force in 
         * RBDL format and expressed in world frame (to be
         * multiplied with the Hessian).
         * @return the Hessian-force product H^T.f 
         * (sizeDOF x sizeDOF).
         */
        Eigen::MatrixXd diffHessianForceInWorldProduct(
            const std::string& frameName,
            const Eigen::Vector3d& forceRBDLInWorld);

        /**
         * Compute the partial derivative of a rotated vector
         * with respect to the degrees of freedom positions.
         * The rotated vector is computed:
         * vectInDst = orientation(srcFrame, dstFrame)*vectInSrc
         * Internal degrees of freedom positions 
         * are used.
         * WARNING: to be correct, internal state must have
         * been updated using updateKinematics().
         *
         * @return the partial derivative matrix M (3xsizeDOF) 
         * such as:
         * d vectInDst = M.dq and
         * vectInDst + (d vectInDst) = R(q + dq).vectInSrc
         */
        Eigen::MatrixXd diffRotatedVector(
            const std::string& srcFrameName,
            const std::string& dstFrameName,
            const Eigen::Vector3d& vectInSrc);

    private:
    
        /**
         * Pinocchio Model instance holding
         * the static model structure description
         */
        pinocchio::Model _model;

        /**
         * Pinocchio Data instance containing
         * algorithms results and temporaries. 
         */
        pinocchio::Data _data;

        /**
         * Current degrees of freedom
         * position and velocity vectors in
         * Pinocchio format.
         * Updated by updateKinematics().
         */
        Eigen::VectorXd _statePos;
        Eigen::VectorXd _stateVel;

        /**
         * Index mapping between RBDL position 
         * (sizeVectPos()) or velocity (sizeVectVel()) 
         * vectors to Pinocchio convention.
         * Mapping: indexInPinocchio -> indexInRBDL.
         */
        std::vector<size_t> _indexPosMapRBDLToPinocchio;
        std::vector<size_t> _indexVelMapRBDLToPinocchio;

        /**
         * Mapping from frame names to Pinocchio 
         * frame indexes
         */
        std::map<std::string, size_t> _mappingFrames;

        /**
         * Convert degree of freedom position or velocity 
         * vectors from RBDL to Pinocchio convention.
         *
         * @param posRBDL Position vector in RBDL
         * format including the floating base 
         * translation and quaternion.
         * @param velRBDL Velocity vector in RBDL
         * format including the floating base linear and 
         * angular velocity.
         * @return the position or velocity vector 
         * in Pinocchio format with FreeFlyer base.
         */
        Eigen::VectorXd convertPosRBDLToPinocchio(
            const Eigen::VectorXd& posRBDL) const;
        Eigen::VectorXd convertVelRBDLToPinocchio(
            const Eigen::VectorXd& velRBDL) const;

        /**
         * Return the Pinocchio frame id associated 
         * to a given name
         */
        size_t getFrameId(const std::string& frameName) const;

        /**
         * @return dummy model to bypass Pinocchio
         * Data required default constructor problem
         */
        static pinocchio::Model getDummyModel();
};

}

#endif

