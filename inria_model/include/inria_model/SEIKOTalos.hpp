#ifndef INRIA_MODEL_SEIKOTALOS_HPP
#define INRIA_MODEL_SEIKOTALOS_HPP

#include <inria_model/SEIKOWrapper.hpp>
#include <inria_maths/Angle.h>

namespace inria {

/**
 * SEIKOTalos
 *
 * Sequential Equilibrium Inverse Kinematic Optimization 
 * configured and specialized for Talos robot.
 */
class SEIKOTalos : public SEIKOWrapper
{
    public:

        /**
         * SEIKO parameters
         */
        double jointPosMargin = DegToRad(15.0);
        double jointVelLimit = 0.2;
        double jointTauLimitRatio = 0.65;
        double jointWeightPos = 10.0;
        double jointWeightVel = 1000.0;
        double jointWeightTauArms = 0.005;
        double jointWeightTauOthers = 0.00002;
        double jointClampPos = DegToRad(10.0);
        double weightPosFoot = 1e5; 
        double weightPosHand = 5e3;
        double weightMatFoot = 1e4;
        double weightMatHand = 1e2;
        double weightWrenchEnabled = 0.002;
        double weightForceEnabled = 0.005;
        double weightWrenchDisabling = 1e4;
        double weightForceDisabling = 1e4;
        double weightForcePushMode = 1e4;
        double forceMinFoot = 100.0;
        double forceMaxFoot = 1000.0;
        double forceMinHand = 10.0;
        double forceMaxHand = 150.0;
        double limitVelForceFoot = 60.0;
        double limitVelTorqueFoot = 0.04*60.0;
        double limitVelForceHand = 10.0;
        double frictionCoef = 0.6;
        double clampPos = 0.04;
        double clampMat = DegToRad(10.0);
        double limitCOPX = 0.04;
        double limitCOPY = 0.03;
        double weightIneqPos = 1e2;
        double weightIneqVel = 1e2;
        double weightIneqForce = 1e2;
        double weightIneqID = 1.0;
        
        /**
         * Pose filter parameters
         */
        bool useRawVRPose = true;
        bool isLocalFrame = false;
        double scalingLin = 1.0;
        double clampRadiusLin = 0.03;
        double clampRadiusAng = 0.2;
        double cutoffFreq = 2.0;
        double maxVelLin = 0.2;
        double maxAccLin = 2.0;
        double maxVelAng = 1.0;
        double maxAccAng = 3.0;

        /**
         * Force velocity command parameters
         */
        double clampRadiusForce = 5.0;
        double maxVelForce = limitVelForceHand;
        double maxAccForce = 10.0*limitVelForceHand;

    public:

        /**
         * Constructor inheritance
         */
        using SEIKOWrapper::SEIKOWrapper;
        
        /**
         * Define and setup contact frames.
         * Must be called only once.
         */
        void setup();

        /**
         * Reset SEIKO and internal state
         * to default.
         *
         * @return true if the reset static ID is successful.
         */
        bool reset();

        /**
         * Read access to frame names for end-effectors
         */
        const std::string& nameFrameFootLeft() const;
        const std::string& nameFrameFootRight() const;
        const std::string& nameFrameHandLeft() const;
        const std::string& nameFrameHandRight() const;

    private:

        /**
         * Feet and hands contact names
         */
        std::string _nameFootLeft;
        std::string _nameFootRight;
        std::string _nameHandLeft;
        std::string _nameHandRight;
};

}

#endif

