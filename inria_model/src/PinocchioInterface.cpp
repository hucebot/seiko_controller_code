#include <stdexcept>
#include <inria_model/PinocchioInterface.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/sample-models.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <inria_maths/Skew.h>

namespace inria {

PinocchioInterface::PinocchioInterface() :
    _model(),
    _data(getDummyModel()),
    _statePos(),
    _stateVel(),
    _indexPosMapRBDLToPinocchio(),
    _indexVelMapRBDLToPinocchio(),
    _mappingFrames()
{
}

PinocchioInterface::PinocchioInterface(
    const Model& modelRBDL) :
    _model(),
    _data(getDummyModel()),
    _indexPosMapRBDLToPinocchio(),
    _indexVelMapRBDLToPinocchio(),
    _mappingFrames()
{
    //Load model structure from URDF
    pinocchio::urdf::buildModel(
        modelRBDL.getPathURDF(),
        pinocchio::JointModelFreeFlyer(),
        _model,
        false);
    //Initialize model data
    _data = pinocchio::Data(_model);

    //Build position and velocity vectors mapping
    size_t sizeDOF = modelRBDL.sizeDOF();
    size_t sizeJoint = modelRBDL.sizeJoint();
    _indexPosMapRBDLToPinocchio.resize(modelRBDL.sizeVectPos());
    _indexVelMapRBDLToPinocchio.resize(modelRBDL.sizeVectVel());
    //Position
    _indexPosMapRBDLToPinocchio[0] = 0;
    _indexPosMapRBDLToPinocchio[1] = 1;
    _indexPosMapRBDLToPinocchio[2] = 2;
    _indexPosMapRBDLToPinocchio[3] = 3;
    _indexPosMapRBDLToPinocchio[4] = 4;
    _indexPosMapRBDLToPinocchio[5] = 5;
    _indexPosMapRBDLToPinocchio[6] = sizeDOF;
    for (size_t i=0;i<sizeJoint;i++) {
        std::string name = modelRBDL.getNameDOF(i+6);
        size_t index = 7 - 2 + _model.getJointId(name);
        _indexPosMapRBDLToPinocchio[index] = i+6;
    }
    //Velocity
    _indexVelMapRBDLToPinocchio[0] = 0;
    _indexVelMapRBDLToPinocchio[1] = 1;
    _indexVelMapRBDLToPinocchio[2] = 2;
    _indexVelMapRBDLToPinocchio[3] = 3;
    _indexVelMapRBDLToPinocchio[4] = 4;
    _indexVelMapRBDLToPinocchio[5] = 5;
    for (size_t i=0;i<sizeJoint;i++) {
        std::string name = modelRBDL.getNameDOF(i+6);
        size_t index = 6 - 2 + _model.getJointId(name);
        _indexVelMapRBDLToPinocchio[index] = i+6;
    }

    //Build frame mappings
    for (size_t i=0;i<_model.frames.size();i++) {
        _mappingFrames.insert(
            std::make_pair(_model.frames[i].name, i));
    }
    _mappingFrames.insert(
        std::make_pair("ROOT", 0));
    _mappingFrames.insert(
        std::make_pair("base", 1));

    //State initialization from 
    //RBDL wrapper model state
    _statePos = 
        convertPosRBDLToPinocchio(modelRBDL.getDOFPosVect());
    _stateVel = 
        convertVelRBDLToPinocchio(modelRBDL.getDOFVelVect());
}

void PinocchioInterface::updateKinematics(
    const Eigen::VectorXd& posRBDL,
    const Eigen::VectorXd& velRBDL)
{
    //Update internal state
    _statePos = convertPosRBDLToPinocchio(posRBDL);
    if (velRBDL.size() == 0) {
        _stateVel.setZero();
    } else {
        _stateVel = convertVelRBDLToPinocchio(velRBDL);
    }

    //Compute forward kinematics and update every frames
    pinocchio::framesForwardKinematics(
        _model, _data, _statePos);
    
    //Compute joint Jacobian
    pinocchio::computeJointJacobians(
        _model, _data);
}

Eigen::Vector3d PinocchioInterface::position(
    const std::string& srcFrameName, 
    const std::string& dstFrameName,
    const Eigen::Vector3d& point)
{
    //Retrieve frame ids
    size_t srcFrameId = getFrameId(srcFrameName);
    size_t dstFrameId = getFrameId(dstFrameName);

    //No transformation if no actual frame change
    if (srcFrameId == dstFrameId) {
        return point;
    }
    
    //Transform from source frame to world frame
    Eigen::Vector3d ptInWorld;
    if (srcFrameId != 0) {
        pinocchio::SE3 poseSrcInWorld = _data.oMf[srcFrameId];
        ptInWorld = 
            poseSrcInWorld.translation() 
            + poseSrcInWorld.rotation()*point;
    } else {
        ptInWorld = point;
    }

    //Transform from world frame to destination frame
    Eigen::Vector3d ptInBase;
    if (dstFrameId != 0) {
        pinocchio::SE3 poseDstInWorld = _data.oMf[dstFrameId];
        ptInBase = 
            poseDstInWorld.rotation().transpose()
            *(ptInWorld-poseDstInWorld.translation());
    } else {
        ptInBase = ptInWorld;
    }

    return ptInBase;
}

Eigen::Matrix3d PinocchioInterface::orientation(
    const std::string& srcFrameName, 
    const std::string& dstFrameName)
{
    //Retrieve frame ids
    size_t srcFrameId = getFrameId(srcFrameName);
    size_t dstFrameId = getFrameId(dstFrameName);

    //No transformation if no actual frame change
    if (srcFrameId == dstFrameId) {
        return Eigen::Matrix3d::Identity();
    }

    //Transform from source frame to world frame
    Eigen::Matrix3d rotSrcToWorld;
    if (srcFrameId != 0) {
        rotSrcToWorld = _data.oMf[srcFrameId].rotation();
    } else {
        rotSrcToWorld.setIdentity();
    }
    
    //Transform from world frame to destination frame
    Eigen::Matrix3d rotDstToWorld;
    if (dstFrameId != 0) {
        rotDstToWorld = _data.oMf[dstFrameId].rotation();
    } else {
        rotDstToWorld.setIdentity();
    }
    
    return rotDstToWorld.transpose()*rotSrcToWorld;
}

Eigen::MatrixXd PinocchioInterface::frameJacobian(
    const std::string& srcFrameName,
    const std::string& dstFrameName)
{
    //Retrieve frame id
    size_t srcFrameId = getFrameId(srcFrameName);
    size_t dstFrameId = getFrameId(dstFrameName);

    //Special case
    if (srcFrameId == 0 && dstFrameId == 0) {
        return Eigen::MatrixXd::Zero(6, _model.nv);
    }

    Eigen::MatrixXd jacPinocchio = Eigen::MatrixXd::Zero(6, _model.nv);
    if (srcFrameId == dstFrameId || dstFrameId == 0) {
        //Compute jacobian with Pinocchio in local body frame
        Eigen::MatrixXd jacPinocchioTmp = Eigen::MatrixXd::Zero(6, _model.nv);
        pinocchio::getFrameJacobian(
            _model, _data, srcFrameId, pinocchio::LOCAL, jacPinocchioTmp);
        //Override the linear part in order to 
        //use RBDL floating base convention
        Eigen::Matrix3d matSrcToWorld = _data.oMf[srcFrameId].rotation();
        jacPinocchioTmp.block(0, 0, 3, 3) = matSrcToWorld.transpose();
        //Convert in RBDL format (angular motion first, linear part second)
        //and transform from local source body frame to destination frame
        Eigen::Matrix3d matSrcToDst = orientation(srcFrameName, dstFrameName);
        jacPinocchio.block(0, 0, 3, _model.nv) = 
            matSrcToDst*jacPinocchioTmp.block(3, 0, 3, _model.nv);
        jacPinocchio.block(3, 0, 3, _model.nv) = 
            matSrcToDst*jacPinocchioTmp.block(0, 0, 3, _model.nv);
    } else {
        //Compute jacobian with Pinocchio in local body frame
        Eigen::MatrixXd srcJ = Eigen::MatrixXd::Zero(6, _model.nv);
        Eigen::MatrixXd dstJ = Eigen::MatrixXd::Zero(6, _model.nv);
        pinocchio::getFrameJacobian(
            _model, _data, srcFrameId, pinocchio::LOCAL, srcJ);
        pinocchio::getFrameJacobian(
            _model, _data, dstFrameId, pinocchio::LOCAL, dstJ);
        //General relative Jacobian case
        Eigen::Matrix3d mat = orientation(srcFrameName, dstFrameName);
        Eigen::Vector3d pos = position(srcFrameName, dstFrameName);
        Eigen::Matrix3d skew = SkewMatrix(-pos);
        jacPinocchio.block(0,0,3,_model.nv) = 
            mat*srcJ.block(3,0,3,_model.nv)
            - dstJ.block(3,0,3,_model.nv);
        jacPinocchio.block(3,0,3,_model.nv) = 
            mat*srcJ.block(0,0,3,_model.nv)
            - dstJ.block(0,0,3,_model.nv)
            - skew*dstJ.block(3,0,3,_model.nv);
        //Override the linear part in order to 
        //use RBDL floating base convention
        Eigen::Matrix3d matSrcToWorld = _data.oMf[srcFrameId].rotation();
        if (srcFrameId == 0) {
            jacPinocchio.block(3,0,3,3) = -mat*matSrcToWorld.transpose();
        }
    }
 
    return jacPinocchio;
}

Eigen::VectorXd PinocchioInterface::gravityVector()
{
    //Compute gravity vector with Pinocchio
    Eigen::VectorXd gPinocchio = 
        pinocchio::computeGeneralizedGravity(_model, _data, _statePos);
    //Convert floating base position frame convention
    Eigen::Matrix3d mat = _data.oMf[2].rotation();
    //Apply rotation from base to world
    gPinocchio.segment(0, 3) = mat*gPinocchio.segment(0, 3);

    return gPinocchio;
}
        
Eigen::MatrixXd PinocchioInterface::massMatrix()
{
    //Compute the Composite Rigid Body Algorithm with Pinocchio
    Eigen::MatrixXd mPinocchio = 
        pinocchio::crbaMinimal(_model, _data, _statePos);
    //Convert floating base position frame convention
    Eigen::Matrix3d mat = _data.oMf[2].rotation();
    //Apply rotation from base to world
    mPinocchio.block(0,3,3,_model.nv-3) = 
        mat*mPinocchio.block(0,3,3,_model.nv-3);
    //Copy to get the full symmetric matrix 
    //from the upper triangular
    mPinocchio.triangularView<Eigen::StrictlyLower>() = 
        mPinocchio.transpose().triangularView<Eigen::StrictlyLower>();

    return mPinocchio;
}
        
Eigen::MatrixXd PinocchioInterface::diffGravityVector()
{
    //Compute gravity vector partial 
    //derivatives with Pinocchio
    Eigen::MatrixXd gPartials = Eigen::MatrixXd::Zero(
        _model.nv, _model.nv);
    pinocchio::computeGeneralizedGravityDerivatives(
        _model, _data, _statePos, gPartials);
    
    //The floating base linear force (expressed in world 
    //frame according to RBDL convention) is not 
    //changed by any degree of freedom (base or joints)
    gPartials.block(0, 0, 3, _model.nv).setZero();

    return gPartials;
}
        
Eigen::MatrixXd PinocchioInterface::diffHessianWrenchInLocalProduct(
    const std::string& frameName,
    const Eigen::Vector6d& wrenchRBDLInLocal)
{
    //Pinocchio frame id
    size_t frameId = getFrameId(frameName);
    //Joint id associated to the contact frame
    size_t jointId = _model.frames[frameId].parent;
    //Transformation from the contact frame 
    //to its associated joint frame
    pinocchio::SE3 transformFrameToJoint = 
        _model.frames[frameId].placement;

    //Convert the wrench from RBDL to Pinocchio
    Eigen::Vector6d wrenchInFrame;
    wrenchInFrame.segment(0, 3) = wrenchRBDLInLocal.segment(3, 3);
    wrenchInFrame.segment(3, 3) = wrenchRBDLInLocal.segment(0, 3);
    //Convert the wrench from contact frame 
    //to associated joint frame
    Eigen::Vector6d wrenchInJoint = 
        transformFrameToJoint.toDualActionMatrix()*wrenchInFrame;

    //Build the set of external wrench applied on joint frames
    pinocchio::container::aligned_vector<pinocchio::Force> externalWrenches(
        _model.joints.size());
    for (size_t i=0;i<_model.joints.size();i++) {
        externalWrenches[i] = Eigen::Vector6d::Zero();
    }
    externalWrenches[jointId] = wrenchInJoint;

    //Disable the gravity for RNEA computation
    _model.gravity.linear().setZero();

    //Compute inverse dynamics partial derivatives
    //See equation 10a from paper 
    //"Analytical Derivatives of Rigid Body Dynamics Algorithms"
    Eigen::MatrixXd partialDq = Eigen::MatrixXd::Zero(_model.nv, _model.nv);
    pinocchio::computeStaticTorqueDerivatives(
        _model, _data, _statePos, externalWrenches, partialDq);
        
    //Inverse sign
    partialDq = -partialDq;
    //Conversion from Pinocchio to RBDL convention.
    //Convert floating base linear forces rows 
    //expressed from local to world frame
    partialDq.block(0, 0, 3, _model.nv) = 
        _data.oMf[2].rotation()*partialDq.block(0, 0, 3, _model.nv);
    //Convert the (0,3,3,3) floating base linear w.r.t angular 
    //forces block from RBDL to Pinocchio convention
    Eigen::Matrix3d transformFrameToBase = 
        _data.oMf[2].rotation().transpose()*_data.oMf[frameId].rotation();
    Eigen::Vector3d tmpForce = 
        transformFrameToBase*wrenchInFrame.segment(0, 3);
    Eigen::Matrix3d tmpSkew = -SkewMatrix(tmpForce);
    partialDq.block(0, 3, 3, 3) = 
        _data.oMf[2].rotation()*tmpSkew;

    //Enable back the gravity to default
    _model.gravity.linear() = pinocchio::Model::gravity981;
    
    return partialDq;
}

Eigen::MatrixXd PinocchioInterface::diffHessianForceInWorldProduct(
    const std::string& frameName,
    const Eigen::Vector3d& forceRBDLInWorld)
{
    //Diff (J_InLocal*force) = Diff(Rot*J_InWorld*force)
    //See research note of 28/11/2019.

    //Compute the Hessian wrench product where the wrench
    //in converted in local body frame
    Eigen::Matrix3d mat = orientation(frameName, "ROOT");
    Eigen::Vector6d wrenchInLocal = Eigen::Vector6d::Zero();
    wrenchInLocal.segment(3, 3) = mat.transpose()*forceRBDLInWorld;
    Eigen::MatrixXd hessianProd = 
        diffHessianWrenchInLocalProduct(frameName, wrenchInLocal);

    //Compute the additional term coming from
    //the differentiation of the rotation matrix
    Eigen::MatrixXd jac = frameJacobian(frameName, "ROOT");
    Eigen::Matrix3d skew = SkewMatrix(forceRBDLInWorld);
    Eigen::MatrixXd termDiffRot = 
        jac.block(3,0,3,_model.nv).transpose()
        *skew
        *jac.block(0,0,3,_model.nv);

    //Return the sum of the two term
    return hessianProd + termDiffRot;
}

Eigen::MatrixXd PinocchioInterface::diffRotatedVector(
    const std::string& srcFrameName,
    const std::string& dstFrameName,
    const Eigen::Vector3d& vectInSrc)
{
    Eigen::Matrix3d mat = orientation(srcFrameName, dstFrameName);
    Eigen::MatrixXd jac = frameJacobian(srcFrameName, dstFrameName);
    Eigen::Vector3d vectInDst = mat*vectInSrc;

    return -SkewMatrix(vectInDst)*jac.block(0,0,3,_model.nv);
}

Eigen::VectorXd PinocchioInterface::convertPosRBDLToPinocchio(
    const Eigen::VectorXd& posRBDL) const
{
    if ((size_t)posRBDL.size() != _indexPosMapRBDLToPinocchio.size()) {
        throw std::logic_error(
            "inria::PinocchioInterface::convertPosRBDLToPinocchio: "
            "Invalid position vector size.");
    }

    Eigen::VectorXd posPinocchio(posRBDL.size());
    for (size_t i=0;i<_indexPosMapRBDLToPinocchio.size();i++) {
        posPinocchio(i) = posRBDL(_indexPosMapRBDLToPinocchio[i]);
    }
    return posPinocchio;
}
Eigen::VectorXd PinocchioInterface::convertVelRBDLToPinocchio(
    const Eigen::VectorXd& velRBDL) const
{
    if ((size_t)velRBDL.size() != _indexVelMapRBDLToPinocchio.size()) {
        throw std::logic_error(
            "inria::PinocchioInterface::convertVelRBDLToPinocchio: "
            "Invalid velocity vector size.");
    }
    
    Eigen::VectorXd velPinocchio(velRBDL.size());
    for (size_t i=0;i<_indexVelMapRBDLToPinocchio.size();i++) {
        velPinocchio(i) = velRBDL(_indexVelMapRBDLToPinocchio[i]);
    }
    return velPinocchio;
}
        
size_t PinocchioInterface::getFrameId(const std::string& frameName) const
{
    if (_mappingFrames.count(frameName) == 0) {
        throw std::logic_error(
            "inria::PinocchioInterface::getFrameId: "
            "Unknown frame name: " + frameName);
    }
    return _mappingFrames.at(frameName);
}
        
pinocchio::Model PinocchioInterface::getDummyModel()
{
    pinocchio::Model tmpModel; 
    pinocchio::buildModels::humanoidRandom(tmpModel);
    return tmpModel;
}

}

