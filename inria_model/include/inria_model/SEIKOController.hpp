#ifndef INRIA_MODEL_SEIKOCONTROLLER_HPP
#define INRIA_MODEL_SEIKOCONTROLLER_HPP

#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include <inria_model/Model.hpp>
#include <inria_model/PinocchioInterface.hpp>

namespace inria {

/**
 * SEIKOController
 *
 * Sequential Equilibrium Inverse Kinematic Optimization
 * for whole body admittance control on position controlled system
 * using explicit flexibility joint model.
 */
class SEIKOController
{
    public:
        
        /**
         * Structure for point (3d) and plane (6d) contacts.
         * WARNING: For point contact, all 3d forces
         * are expressed in the contact frame defined by contactMat.
         * Contact frame is defined with respect to world frame and is assumed
         * fixed as long as the contact is enabled.
         */
        struct ContactPoint_t {
            /**
             * Configuration
             */
            //Cartesian frame id and name
            size_t frameId;
            std::string frameName;
            //Weight for force change target
            Eigen::Vector3d weightForce;
            //Weight for effector position task when disabled
            double weightPos;
            //Weight for effector orientation when enabled
            double weightMat;
            /**
             * Input
             */
            //If true, force is assumed 
            //to be applied on this contact
            bool isEnabled;
            //Contact surface orientation expressed in world 
            //frame (aligning Z axis as surface normal)
            Eigen::Matrix3d contactMat;
            //Desired contact force change
            Eigen::Vector3d targetDeltaForce;
            /**
             * State
             */
            //Integrated estimated force solution
            Eigen::Vector3d stateForce;
            /**
             * Cache
             */
            //Cache desired Cartesian pose
            Eigen::Vector3d targetPos;
            Eigen::Matrix3d targetMat;
            //Cache Jacobian computed with corrected model
            Eigen::MatrixXd jacWorld;
            //Cache last computed force change
            Eigen::Vector3d solutionForce;
        };
        struct ContactPlane_t {
            /**
             * Configuration
             */
            //Cartesian frame id and name
            size_t frameId;
            std::string frameName;
            //Weight for wrench change target
            Eigen::Vector6d weightWrench;
            //Weight for effector pose task when disabled
            double weightPos;
            double weightMat;
            /**
             * Input
             */
            //If true, wrench is assumed 
            //to be applied on this contact
            bool isEnabled;
            //Desired contact wrench change
            Eigen::Vector6d targetDeltaWrench;
            /**
             * State
             */
            //Integrated estimated wrench solution
            Eigen::Vector6d stateWrench;
            /**
             * Cache
             */
            //Cache desired Cartesian pose
            Eigen::Vector3d targetPos;
            Eigen::Matrix3d targetMat;
            //Cache Jacobian computed with corrected model
            Eigen::MatrixXd jacBody;
            Eigen::MatrixXd jacWorld;
            //Cache last computed wrench change
            Eigen::Vector6d solutionWrench;
        };
        /**
         * Structure for posture and joint quantities.
         */
        struct Configuration_t {
            /**
             * Configuration
             */
            //If false, state and command integration is disabled
            //(only for runSEIKO, not runInit)
            bool isIntegrating;
            //Gain applied on disabled effectors pose task delta
            double gainPose;
            //Penalty weight on floating base change regularization
            double weightRegBase;
            //Weight on joint elastic energy minimization
            double weightElasticEnergy;
            //Penalty weight on joint command change regularization (sizeJoint())
            Eigen::VectorXd weightRegCmdChange;
            //Penalty weight on joint command offset regularization (sizeJoint())
            Eigen::VectorXd weightRegCmdOffset;
            //Joint stiffness default gain to convert from
            //joint position to joint torque (sizeJoint())
            Eigen::VectorXd stiffnessJoint;
            //Joint lower and upper angle position limits (sizeJoint())
            Eigen::VectorXd limitPosLowerJoint;
            Eigen::VectorXd limitPosUpperJoint;
            //Joint absolute position command offset limit
            double limitPosCmdOffset;
            //Joint absolute torque limits (sizeJoint())
            Eigen::VectorXd limitTauJoint;
            //Ratio margin for stiffness joint estimation limits
            double limitRatioStiffness;
            /**
             * Input
             */
            //Desired posture from SEIKO is given by the model
            /**
             * State
             */
            //Integrated estimated posture solution 
            //under flexibility (sizeVectPos())
            Eigen::VectorXd statePosture;
            //Integrated joint position offset on command (sizeJoint())
            Eigen::VectorXd cmdOffsetJoint;
            //Integrated joint stiffness gain (sizeJoint())
            Eigen::VectorXd stateStiffnessJoint;
            //Integrated additional base external wrench ([lin, ang])
            Eigen::Vector6d stateExternalWrench;
            /**
             * Cache
             */
            //Floating base and joint position desired command 
            //retrieved from model (sizeVectPos())
            Eigen::VectorXd targetPosture;
            //Last computed posture change solution (sizeVectVel())
            Eigen::VectorXd solutionPosture;
            //Last computed command position offset change (sizeJoint())
            Eigen::VectorXd solutionCommand;
            //Last computed resulting joint command with 
            //desired position added to command offset
            Eigen::VectorXd commandJoint;
            //Last computed state joint torque
            Eigen::VectorXd torqueJoint;
            //Last computed joint stiffness gain change (sizeJoint())
            Eigen::VectorXd solutionStiffnessJoint;
            //Last computed base external wrench change ([lin, ang])
            Eigen::Vector6d solutionExternalWrench;
            //Last computed bias term norms for floating base 
            //and joint parts of the equilibrium equation
            double biasBase;
            double biasJoint;
        };
        
        /**
         * Empty initialization.
         */
        SEIKOController();
        
        /**
         * Initialization with a Model.
         * The model instance must be valid during
         * the whole use of SEIKOController.
         *
         * @param model Reference to the model
         * containing the desired posture previously computed
         * by multi-contact SEIKO.
         */
        SEIKOController(Model& model);
        
        /**
         * Define a new potential contact plane
         * (6-DOFs) or contact point (3-DOFs) 
         * with default configuration.
         *
         * @param frameName Model frame name in kinematics tree.
         */
        void addContactPoint(const std::string& frameName);
        void addContactPlane(const std::string& frameName);

        /**
         * Reset controller internal state
         */
        void reset();

        /**
         * Set given effector as enabled
         * @param frameName Model frame name of previously defined contact.
         */
        void askSwitchingEnable(const std::string& frameName);

        /**
         * Return read write reference toward point or plane 
         * structure from its contact name
         * @param frameName Model frame name of previously defined contact.
         */
        const ContactPoint_t& getPoint(const std::string& frameName) const;
        ContactPoint_t& getPoint(const std::string& frameName);
        const ContactPlane_t& getPlane(const std::string& frameName) const;
        ContactPlane_t& getPlane(const std::string& frameName);

        /**
         * Return read write reference toward configuration with
         * posture and joint structure
         */
        const Configuration_t& getConfiguration() const;
        Configuration_t& getConfiguration();
        
        /**
         * Solve the quadratic program of the SQP computing 
         * the flexibility posture from given fixed joint command.
         * Used parameters:
         * weightRegBase, weightElasticEnergy, weightForce, weightWrench.
         *
         * @return true if the QP succeed, else
         * false if no feasible solution is found.
         */
        bool runInit();

        /**
         * Solve the quadratic program
         * using reduced fast formulation.
         * Used parameters:
         * weightRegBase, weightRegCmdChange, weightRegCmdOffset,
         * weightForce, weightWrench.
         *
         * @param dt Time step in seconds.
         * @return true if the QP succeed, else
         * false if no feasible solution is found.
         */
        bool runSEIKO(double dt);

    private:
        
        /**
         * Pointer to model instance
         * containing desired posture
         */
        Model* _model;
        
        /**
         * Pinocchio model interface instance
         */
        PinocchioInterface _pinocchio;
        
        /**
         * Container for defined Cartesian 
         * contact planes and points and 
         * mapping from frame name to index
         */
        std::vector<ContactPoint_t> _contactPoint;
        std::vector<ContactPlane_t> _contactPlane;
        std::map<std::string, size_t> _mappingPoint;
        std::map<std::string, size_t> _mappingPlane;

        /**
         * Posture and joint configuration
         */
        Configuration_t _configuration;
};

}

#endif

