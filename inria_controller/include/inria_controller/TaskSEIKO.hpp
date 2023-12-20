#ifndef INRIA_CONTROLLER_TASKSEIKO_HPP
#define INRIA_CONTROLLER_TASKSEIKO_HPP

#include <Eigen/Dense>
#include <RhIO.hpp>
#include <inria_controller/TaskBaseTalos.hpp>
#include <inria_model/Model.hpp>
#include <inria_model/SEIKOTalos.hpp>
#include <inria_model/SEIKOController.hpp>
#include <inria_maths/FilterExponential.hpp>
#include <inria_maths/TrajectoryBangBangAcc.hpp>
#include <inria_maths/FilterDifferentiator2.hpp>
#include <inria_maths/FilterInterpolation.hpp>

namespace inria {

/**
 * TaskSEIKO
 *
 * Whole body position control using SEIKO
 * for Talos robot and whole body admittance
 * stabilization controller using flexiblity model
 */
class TaskSEIKO : public TaskBaseTalos
{
    protected:
        
        /**
         * Task name defining the task.
         */
        virtual const char* name() const override;
        
        /**
         * Task defined task scheduling frequency
         * (by thread manager)
         *
         * @return desire frequency in Hertz
         */
        virtual double schedulingFrequency() const override;
        
        /**
         * Task defined prediction ahead time step 
         * for lowlevel interpolation.
         */
        virtual double aheadTimeStep() const override;

        /**
         * Task defined no RT task initialization
         * run at controller initialization.
         */
        virtual void init(
            RhIO::IONode& rhioNode,
            const std::string& pathModel) override;

        /**
         * Task defined RT update step.
         */
        virtual void update(
            double dt_task,
            double dt_ahead,
            const ControllerTalosState_t& state,
            ControllerTalosCommand_t& command) override;

    public:
        
        /**
         * Structure for exposing SEIKO end-effectors 
         * command and monitoring to RhIO
         */
        struct Effector_t {
            //Effector contact type
            bool isPoint;
            //Command inputs
            RhIO::WrapperBool askSwitchEnable;
            RhIO::WrapperBool askSwitchDisable;
            RhIO::WrapperBool cmdIsPushMode;
            RhIO::WrapperFloat cmdVelForceNormal;
            RhIO::WrapperBool cmdIsClutch;
            RhIO::WrapperVect3d cmdRawPos;
            RhIO::WrapperVect4d cmdRawQuat;
            RhIO::WrapperVect3d cmdVelLin;
            RhIO::WrapperVect3d cmdVelAng;
            //Velocity admittance applied
            RhIO::WrapperVect3d admVelLin;
            RhIO::WrapperVect3d admVelAng;
            //Monitoring SEIKO state
            RhIO::WrapperBool isEnabled;
            RhIO::WrapperVect3d targetPos;
            RhIO::WrapperVect3d goalPos;
            RhIO::WrapperVect4d contactQuat;
            //Monitoring controller state
            RhIO::WrapperVect3d deltaForce;
            RhIO::WrapperVect3d deltaTorque;
            RhIO::WrapperVect3d goalForce;
            RhIO::WrapperVect3d goalTorque;
            RhIO::WrapperVect3d readForce;
            RhIO::WrapperVect3d readTorque;
            RhIO::WrapperVect3d filteredForce;
            RhIO::WrapperVect3d filteredTorque;
            RhIO::WrapperVect3d velForce;
            RhIO::WrapperVect3d velTorque;
            RhIO::WrapperVect3d errorForce;
            RhIO::WrapperVect3d errorTorque;
            RhIO::WrapperVect3d effortForce;
            RhIO::WrapperVect3d effortTorque;
            RhIO::WrapperVect3d solForce;
            RhIO::WrapperVect3d solTorque;
            RhIO::WrapperVect3d stateForce;
            RhIO::WrapperVect3d stateTorque;
        };
        
        /**
         * Integrated goal and measured models
         */
        Model _modelGoal;
        Model _modelRead;
        
        /**
         * SEIKO Retargeting for Talos robot
         */
        SEIKOTalos _retargeting;
        
        /**
         * Whole body feet and hands wrench estimation
         * from FT and joint torque sensors and velocity
         * differentiation filter
         */
        FilterDifferentiator2<Eigen::Vector6d> _filterVelFootLeft;
        FilterDifferentiator2<Eigen::Vector6d> _filterVelFootRight;
        FilterDifferentiator2<Eigen::Vector3d> _filterVelHandLeft;
        FilterDifferentiator2<Eigen::Vector3d> _filterVelHandRight;

        /**
         * Whole body admittance controller with 
         * flexibility model
         */
        SEIKOController _controller;
        
        /**
         * Map container for RhIO end-effector wrappers
         */
        std::map<std::string, Effector_t> _effectors;
        
        /**
         * State and parameters for measured wrench
         * and joint torque complimentary filtering
         */
        bool _filteredComplimentaryIsInit;
        Eigen::Vector6d _lastDeltaWrenchLeft;
        Eigen::Vector6d _lastDeltaWrenchRight;
        Eigen::Vector3d _lastDeltaForceLeft;
        Eigen::Vector3d _lastDeltaForceRight;
        Eigen::Vector6d _filteredWrenchLeft;
        Eigen::Vector6d _filteredWrenchRight;
        Eigen::Vector3d _filteredForceLeft;
        Eigen::Vector3d _filteredForceRight;
        RhIO::WrapperFloat _rhioParamCutoffFreqComplimentary;
        
        /**
         * Controller parameters
         */
        RhIO::WrapperBool _rhioParamControllerEnabled;
        RhIO::WrapperFloat _rhioParamGainP;
        RhIO::WrapperFloat _rhioParamGainD;
        RhIO::WrapperFloat _rhioParamMaxTauRatio;
        RhIO::WrapperFloat _rhioParamMaxPosCmdOffset;
        
        /**
         * Effector velocity admittance parameters
         */
        RhIO::WrapperBool _rhioParamAdmEnabled;
        RhIO::WrapperFloat _rhioParamAdmPointLinDeadbband;
        RhIO::WrapperFloat _rhioParamAdmPointLinGain;
        RhIO::WrapperFloat _rhioParamAdmPointLinMax;
        RhIO::WrapperFloat _rhioParamAdmPlaneLinDeadbband;
        RhIO::WrapperFloat _rhioParamAdmPlaneLinGain;
        RhIO::WrapperFloat _rhioParamAdmPlaneLinMax;
        RhIO::WrapperFloat _rhioParamAdmPlaneAngDeadbband;
        RhIO::WrapperFloat _rhioParamAdmPlaneAngGain;
        RhIO::WrapperFloat _rhioParamAdmPlaneAngMax;

        /**
         * Additional monitoring
         */
        RhIO::WrapperBool _rhioIsExperiment;
        RhIO::WrapperFloat _rhioDurationRetargeting;
        RhIO::WrapperFloat _rhioDurationController;
        RhIO::WrapperArray _rhioGoalModel;
        RhIO::WrapperArray _rhioReadModel;
        RhIO::WrapperArray _rhioFlexModel;
        RhIO::WrapperArray _rhioCmdOffset;
        RhIO::WrapperArray _rhioStiffnessRatio;
        RhIO::WrapperArray _rhioJointTauGoalRatio;
        RhIO::WrapperArray _rhioJointTauReadRatio;
        RhIO::WrapperArray _rhioJointTauLimitRatio;
        
        /**
         * Final command interpolation filter
         */
        FilterInterpolation<Eigen::VectorXd> _filterInterpolation;
};

}

#endif

