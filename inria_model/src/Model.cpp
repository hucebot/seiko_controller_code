#include <fstream>
#include <stack>
#include <stdexcept>
#include <cmath>
#include <inria_model/Model.hpp>
#include <inria_model/RBDLRootUpdate.h>
#include <inria_maths/Skew.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <urdf/model.h>

namespace inria {
        
Model::Model() :
    _model(),
    _pathURDF(""),
    _baseFrameId(-1),
    _baseFrameName(""),
    _isDirty(false),
    _position(),
    _velocity(),
    _limitPosLower(),
    _limitPosUpper(),
    _limitVelocity(),
    _limitEffort(),
    _dofIndexToName(),
    _dofNameToIndex(),
    _visuals()
{
}

Model::Model(
    const std::string& filename, 
    const std::string& rootFrameName) :
    _model(),
    _pathURDF(filename),
    _baseFrameId(-1),
    _baseFrameName(""),
    _isDirty(false),
    _position(),
    _velocity(),
    _limitPosLower(),
    _limitPosUpper(),
    _limitVelocity(),
    _limitEffort(),
    _dofIndexToName(),
    _dofNameToIndex(),
    _visuals()
{
    //Call actual initialization
    initFromFile(filename, rootFrameName);
}

void Model::initFromFile(
    const std::string& filename,
    const std::string& rootFrameName)
{
    //Read whole file content as string
    std::ifstream file(filename);
    if (!file.is_open() || !file.good()) {
        throw std::runtime_error(
            "inria::Model: Unable to load URDF file: " 
            + filename);
    }
    std::stringstream buffer;
    buffer << file.rdbuf();

    //Forward to actual implementation
    initFromString(buffer.str(), rootFrameName);
}
void Model::initFromString(
    const std::string& fileContent,
    const std::string& rootFrameName)
{
    //Load raw URDF model
    urdf::Model urdf_model;
    if (!urdf_model.initString(fileContent)) {
        throw std::runtime_error(
            "inria::Model: Unable to load URDF content.");
    }
    
    //Load temporary RBDL model 
    //from URDF without floating base
    RBDL::Model modelTmp;
    if (
        !RBDL::Addons::URDFReadFromString(
            fileContent.c_str(), &modelTmp, false, false)
    ) {
        throw std::runtime_error(
            "inria::Model: Unable to load model from URDF.");
    }
    
    //Reconstruct the RBDL tree model to change
    //the body root of the kinematics structure and add
    //the 6 DoFs (translation, spherical rotation) floating base.
    if (modelTmp.mBodyNameMap.count(rootFrameName) == 0) {
        throw std::logic_error(
            "inria::Model: Root body name unknown: " 
            + rootFrameName);
    }
    //Mapping of joint indexes from before to 
    //after root change
    std::vector<size_t> indexesNewToOld;
    _model = RBDLRootUpdate(
        modelTmp, 
        modelTmp.mBodyNameMap.at(rootFrameName), 
        true,
        indexesNewToOld);
    
    //Hold old (before root change) index to 
    //name mapping in RBDL model
    std::vector<std::string> tmpIndexToName;
    //Retrieve all joint names from URDF in depth-first 
    //order so as to match RBDL order (in not updated root model)
//if ROS version >= ROS_MELODIC
#if ROS_VERSION_MINIMUM(1, 14, 0)
    typedef std::shared_ptr<urdf::Link> Tmp_Type_Link;
    typedef std::shared_ptr<urdf::Joint> Tmp_Type_Joint;
#else
    typedef boost::shared_ptr<urdf::Link> Tmp_Type_Link;
    typedef boost::shared_ptr<urdf::Joint> Tmp_Type_Joint;
#endif
    std::map<std::string, Tmp_Type_Link> link_map
        = urdf_model.links_;
    //Holds the links that we are processing in our depth first 
    //traversal with the top element being the current link
    std::stack<Tmp_Type_Link> link_stack;
    //Holds the child joint index of the current link
    std::stack<int> joint_index_stack;
    //Add the root
    link_stack.push(link_map[urdf_model.getRoot()->name]);
    if (link_stack.top()->child_joints.size() > 0) {
        joint_index_stack.push(0);
    }
    //Add the bodies in a depth-first 
    //order of the model tree
    while (link_stack.size() > 0) {
        Tmp_Type_Link cur_link = link_stack.top();
        int joint_idx = joint_index_stack.top();
        if (joint_idx < (int)cur_link->child_joints.size()) {
            Tmp_Type_Joint cur_joint = 
                cur_link->child_joints[joint_idx];
            joint_index_stack.pop();
            joint_index_stack.push(joint_idx+1);
            link_stack.push(link_map[cur_joint->child_link_name]);
            joint_index_stack.push(0);
            //Retrieve revolute and prismatic joints
            if (
                cur_joint->type == urdf::Joint::REVOLUTE ||
                cur_joint->type == urdf::Joint::PRISMATIC
            ) {
                //Build forward and inverse mapping
                tmpIndexToName.push_back(cur_joint->name);
            }
        } else {
            link_stack.pop();
            joint_index_stack.pop();
        }
    }

    //Retrieve links visual data from URDF
    for (const auto& it : urdf_model.links_) {
        //Load mesh data
        if (
            it.second != nullptr && 
            it.second->visual != nullptr &&
            it.second->visual->geometry != nullptr &&
            it.second->visual->geometry->type == urdf::Geometry::MESH
        ) {
            const urdf::Mesh* tmpPtr = dynamic_cast<const urdf::Mesh*>(
                it.second->visual->geometry.get());
            if (tmpPtr != nullptr) {
                Eigen::Vector3d tmpPos;
                Eigen::Vector3d tmpRot;
                Eigen::Vector3d tmpScale;
                tmpPos.x() = 
                    it.second->visual->origin.position.x;
                tmpPos.y() = 
                    it.second->visual->origin.position.y;
                tmpPos.z() = 
                    it.second->visual->origin.position.z;
                it.second->visual->origin.rotation.getRPY(
                    tmpRot.x(), tmpRot.y(), tmpRot.z());
                tmpScale.x() = tmpPtr->scale.x;
                tmpScale.y() = tmpPtr->scale.y;
                tmpScale.z() = tmpPtr->scale.z;
                Eigen::Vector4d color = Eigen::Vector4d::Zero();
                bool isColor = false;
                if (it.second->visual->material != nullptr) {
                    isColor = true;
                    color(0) = it.second->visual->material->color.r;
                    color(1) = it.second->visual->material->color.g;
                    color(2) = it.second->visual->material->color.b;
                    color(3) = it.second->visual->material->color.a;
                }
                _visuals.insert(std::make_pair(it.first, Visual{
                    tmpPos,
                    tmpRot,
                    tmpScale,
                    tmpPtr->filename,
                    isColor,
                    color
                }));
            }
        }
    }

    //Build joints name to index mapping
    //taking into account the change of RBDL body root
    //Initialize floating base DoF names mapping
    _dofIndexToName.clear();
    _dofIndexToName.push_back("basePosX");
    _dofIndexToName.push_back("basePosY");
    _dofIndexToName.push_back("basePosZ");
    _dofIndexToName.push_back("baseQuatX");
    _dofIndexToName.push_back("baseQuatY");
    _dofIndexToName.push_back("baseQuatZ");
    //Convert from old joint index mapping 
    //(before root change) to new mapping
    for (size_t i=0;i<indexesNewToOld.size();i++) {
        if (
            indexesNewToOld.size() != tmpIndexToName.size() ||
            indexesNewToOld[i] >= tmpIndexToName.size()
        ) {
            //Sanity check. This should never happen.
            //Verbose info
            std::cout 
                << "Joint new index to old: size=" 
                << indexesNewToOld.size() 
                << std::endl;
            for (size_t i=0;i<indexesNewToOld.size();i++) {
                std::cout << i << " -> " 
                    << indexesNewToOld[i] << std::endl;
            }
            std::cout 
                << "Joint old index to name: size=" 
                << tmpIndexToName.size() 
                << std::endl;
            for (size_t i=0;i<tmpIndexToName.size();i++) {
                std::cout << i << " -> " 
                    << tmpIndexToName[i] << std::endl;
            }
            throw std::logic_error(
                "inria::Model: Inconsistency in RBDL joint index mapping");
        }
        _dofIndexToName.push_back(tmpIndexToName[indexesNewToOld[i]]);
    }
    //Add last floating base quaternion element
    _dofIndexToName.push_back("baseQuatW");

    //Build inverse map
    _dofNameToIndex.clear();
    for (size_t i=0;i<_dofIndexToName.size();i++) {
        _dofNameToIndex.insert({_dofIndexToName[i], i});
    }

    //Reset DoF position and limit vectors
    _position = Eigen::VectorXd::Zero(_dofNameToIndex.size());
    _position(_dofNameToIndex.at("baseQuatW")) = 1.0;
    _velocity = Eigen::VectorXd::Zero(_dofNameToIndex.size()-1);
    _limitPosLower = Eigen::VectorXd::Zero(_dofNameToIndex.size());
    _limitPosUpper = Eigen::VectorXd::Zero(_dofNameToIndex.size());
    _limitVelocity = Eigen::VectorXd::Zero(_dofNameToIndex.size());
    _limitEffort = Eigen::VectorXd::Zero(_dofNameToIndex.size());

    //Load joint limits from URDF file
    //(skip floating base DoFs)
    for (size_t i=6;i<_dofIndexToName.size()-1;i++) {
        _limitPosLower(i) = 
            urdf_model.getJoint(_dofIndexToName[i])->limits->lower;
        _limitPosUpper(i) = 
            urdf_model.getJoint(_dofIndexToName[i])->limits->upper;
        _limitVelocity(i) = 
            urdf_model.getJoint(_dofIndexToName[i])->limits->velocity;
        _limitEffort(i) = 
            urdf_model.getJoint(_dofIndexToName[i])->limits->effort;
    }

    //Save base root frame id and name
    _baseFrameId = getIndexFrame("base");
    _baseFrameName = rootFrameName;
    //Set default gravity
    setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    //Update internal state
    updateState();
}

size_t Model::sizeDOF() const
{
    if (_dofNameToIndex.size() == 0) {
        return 0;
    } else {
        return _dofNameToIndex.size()-1;
    }
}
size_t Model::sizeJoint() const
{
    if (_dofNameToIndex.size() < 7) {
        return 0;
    } else {
        return _dofNameToIndex.size() - 7;
    }
}
size_t Model::sizeVectPos() const
{
    return _position.size();
}
size_t Model::sizeVectVel() const
{
    return _velocity.size();
}

const std::map<std::string, size_t>& Model::getMappingDOFs() const
{
    return _dofNameToIndex;
}
const std::map<std::string, unsigned int>& Model::getMappingFrames() const
{
    return _model.mBodyNameMap;
}
        
const std::map<std::string, Model::Visual>& Model::getMappingVisuals() const
{
    return _visuals;
}

unsigned int Model::getIndexDOF(const std::string& name) const
{
    if (getMappingDOFs().count(name) == 0) {
        throw std::logic_error(
            "inria::Model::getIndexDOF: Invalid DOF name: " 
            + name);
    }
    return getMappingDOFs().at(name);
}
unsigned int Model::getIndexJoint(const std::string& name) const
{
    if (getMappingDOFs().count(name) == 0) {
        throw std::logic_error(
            "inria::Model::getIndexJoint: Invalid DOF name: " 
            + name);
    }
    size_t index = getMappingDOFs().at(name);
    if (index < 6 || index-6 >= sizeJoint()) {
        throw std::logic_error(
            "inria::Model::getIndexJoint: Invalid joint name: " 
            + name);
    }
    return index-6;
}
unsigned int Model::getIndexFrame(const std::string& name) const
{
    if (getMappingFrames().count(name) == 0) {
        throw std::logic_error(
            "inria::Model::getIndexFrame: Invalid frame name: " 
            + name);
    }
    return getMappingFrames().at(name);
}
        
const std::string& Model::getNameDOF(size_t index) const
{
    if (index >= sizeVectPos()) {
        throw std::logic_error(
            "inria::Model::getNameDOF: Invalid index.");
    }
    return _dofIndexToName[index];
}
const std::string& Model::getNameJoint(size_t index) const
{
    if (index >= sizeJoint()) {
        throw std::logic_error(
            "inria::Model::getNameJoint: Invalid index.");
    }
    return _dofIndexToName[index+6];
}
const std::string& Model::getNameFrame(size_t index) const
{
    for (const auto& it : getMappingFrames()) {
        if (it.second == index) {
            return it.first;
        }
    }
    throw std::logic_error(
        "inria::Model::getNameFrame: Invalid index.");
}

double Model::getDOFPos(size_t index) const
{
    if (index >= sizeVectPos()) {
        throw std::logic_error(
            "inria::Model::getDOFPos: Invalid index: " 
            + std::to_string(index));
    }
    return _position(index);
}
void Model::setDOFPos(size_t index, double val)
{
    if (index >= sizeVectPos()) {
        throw std::logic_error(
            "inria::Model::setDOFPos: Invalid index: " 
            + std::to_string(index));
    }
    _position(index) = val;
    _isDirty = true;
}
double Model::getDOFPos(const std::string& name) const
{
    if (_dofNameToIndex.count(name) == 0) {
        throw std::logic_error(
            "inria::Model::getDOFPos: Invalid name: " 
            + name);
    }
    return _position(_dofNameToIndex.at(name));
}
void Model::setDOFPos(const std::string& name, double val)
{
    if (_dofNameToIndex.count(name) == 0) {
        throw std::logic_error(
            "inria::Model::setDOFPos: Invalid name: " 
            + name);
    }
    _position(_dofNameToIndex.at(name)) = val;
    _isDirty = true;
}

double Model::getDOFVel(size_t index) const
{
    if (index >= sizeVectVel()) {
        throw std::logic_error(
            "inria::Model::getDOFVel: Invalid index: " 
            + std::to_string(index));
    }
    return _velocity(index);
}
void Model::setDOFVel(size_t index, double val)
{
    if (index >= sizeVectVel()) {
        throw std::logic_error(
            "inria::Model::setDOFVel: Invalid index: " 
            + std::to_string(index));
    }
    _velocity(index) = val;
    _isDirty = true;
}
double Model::getDOFVel(const std::string& name) const
{
    if (_dofNameToIndex.count(name) == 0) {
        throw std::logic_error(
            "inria::Model::getDOFVel: Invalid name: " 
            + name);
    }
    return _velocity(_dofNameToIndex.at(name));
}
void Model::setDOFVel(const std::string& name, double val)
{
    if (_dofNameToIndex.count(name) == 0) {
        throw std::logic_error(
            "inria::Model::setDOFVel: Invalid name: " 
            + name);
    }
    _velocity(_dofNameToIndex.at(name)) = val;
    _isDirty = true;
}

Eigen::VectorBlock<const Eigen::VectorXd, -1> 
    Model::getJointPosVect() const
{
    if (sizeJoint() == 0) {
        return Eigen::VectorBlock<const Eigen::VectorXd, -1>(
            _position, 0, 0);
    } else {
        return _position.segment(6, sizeJoint());
    }
}
void Model::setJointPosVect(const Eigen::VectorXd& vect)
{
    if ((size_t)vect.size() != sizeJoint()) {
        throw std::logic_error(
            "inria::Model::setJointPosVect: Invalid vector size.");
    }
    _position.segment(6, sizeJoint()) = vect;
    _isDirty = true;
}

Eigen::VectorBlock<const Eigen::VectorXd, -1> 
    Model::getJointVelVect() const
{
    if (sizeJoint() == 0) {
        return Eigen::VectorBlock<const Eigen::VectorXd, -1>(
            _velocity, 0, 0);
    } else {
        return _velocity.segment(6, sizeJoint());
    }
}
void Model::setJointVelVect(const Eigen::VectorXd& vect)
{
    if ((size_t)vect.size() != sizeJoint()) {
        throw std::logic_error(
            "inria::Model::setJointVelVect: Invalid vector size.");
    }
    _velocity.segment(6, sizeJoint()) = vect;
    _isDirty = true;
}
        
const Eigen::VectorXd& Model::getDOFPosVect() const
{
    return _position;
}
        
void Model::setDOFPosVect(const Eigen::VectorXd& vect)
{
    if ((size_t)vect.size() != sizeVectPos()) {
        throw std::logic_error(
            "inria::Model::setDOFPosVect: Invalid vector size.");
    }
    _position = vect;
    _isDirty = true;
}

Eigen::Vector3d Model::getBasePos() const
{
    if (_position.size() < 3) {
        throw std::logic_error(
            "inria::Model::getBasePos: Not initialized.");
    }
    return _position.segment(0, 3);
}
void Model::setBasePos(const Eigen::Vector3d& pos)
{
    if (_position.size() < 3) {
        throw std::logic_error(
            "inria::Model::setBasePos: Not initialized.");
    }
    _position.segment(0, 3) = pos;
    _isDirty = true;
}
Eigen::Quaterniond Model::getBaseQuat() const
{
    if (_position.size() < 7) {
        throw std::logic_error(
            "inria::Model::getBaseQuat: Not initialized.");
    }
    return Eigen::Quaterniond(
        _position(_position.size()-1),
        _position(3),
        _position(4),
        _position(5));
}
void Model::setBaseQuat(const Eigen::Quaterniond& quat)
{
    if (_position.size() < 6) {
        throw std::logic_error(
            "inria::Model::setBaseQuat: Not initialized.");
    }
    _position(_position.size()-1) = quat.w();
    _position(3) = quat.x();
    _position(4) = quat.y();
    _position(5) = quat.z();
    _isDirty = true;
}

Eigen::Vector3d Model::getBaseVelLinear() const
{
    if (_velocity.size() < 6) {
        throw std::logic_error(
            "inria::Model::getBaseVelLinear: Not initialized.");
    }
    return _velocity.segment(0, 3);
}
void Model::setBaseVelLinear(const Eigen::Vector3d& vel)
{
    if (_velocity.size() < 6) {
        throw std::logic_error(
            "inria::Model::setBaseVelLinear: Not initialized.");
    }
    _velocity.segment(0, 3) = vel;
    _isDirty = true;
}
Eigen::Vector3d Model::getBaseVelAngular() const
{
    if (_velocity.size() < 6) {
        throw std::logic_error(
            "inria::Model::getBaseVelAngular: Not initialized.");
    }
    return _velocity.segment(3, 3);
}
void Model::setBaseVelAngular(const Eigen::Vector3d& vel)
{
    if (_velocity.size() < 6) {
        throw std::logic_error(
            "inria::Model::setBaseVelAngular: Not initialized.");
    }
    _velocity.segment(3, 3) = vel;
    _isDirty = true;
}
        
const Eigen::VectorXd& Model::getDOFVelVect() const
{
    return _velocity;
}
        
void Model::setDOFVelVect(const Eigen::VectorXd& vect)
{
    if ((size_t)vect.size() != sizeVectVel()) {
        throw std::logic_error(
            "inria::Model::setDOFVelVect: Invalid vector size.");
    }
    _velocity = vect;
    _isDirty = true;
}

void Model::importState(Model& model)
{
    //Copy position and velocity state from 
    //given model to this based on names matching
    size_t sizeVel = sizeVectVel();
    for (const auto& it : _dofNameToIndex) {
        const std::string& name = it.first;
        size_t indexSelf = this->_dofNameToIndex.at(name);
        size_t indexOther = model._dofNameToIndex.at(name);
        //Update position
        this->_position(indexSelf) = model._position(indexOther);
        //Update velocity but not the "baseQuatW" element
        //located at the end of RBDL position vector
        if (indexSelf < sizeVel && indexOther < sizeVel) {
            this->_velocity(indexSelf) = model._velocity(indexOther);
        } 
    }

    //If the two models have a different floating base
    //frame, the base position and velocity must be converted
    if (this->_baseFrameName != model._baseFrameName) {
        Eigen::Vector3d baseLinPosInWorld = 
            model.position(_baseFrameName, "ROOT");
        Eigen::Matrix3d baseMatPosInWorld = 
            model.orientation(_baseFrameName, "ROOT");
        Eigen::Vector3d baseLinVelInWorld = 
            model.pointVelocity(_baseFrameName, "ROOT").segment(3, 3);
        Eigen::Vector3d baseAngVelInBody = 
            model.pointVelocity(_baseFrameName, _baseFrameName).segment(0, 3);
        Eigen::Quaterniond baseQuatPosInWorld(baseMatPosInWorld);
        this->setBasePos(baseLinPosInWorld);
        this->setBaseQuat(baseQuatPosInWorld);
        this->setBaseVelLinear(baseLinVelInWorld);
        this->setBaseVelAngular(baseAngVelInBody);
    }
    
    _isDirty = true;
}
 
bool Model::isDirty() const
{
    return _isDirty;
}
        
void Model::updateState()
{
    //Update with position and velocity vector
    RBDL::UpdateKinematicsCustom(
        _model, &_position, &_velocity, nullptr);
    _isDirty = false;
}
        
Eigen::VectorBlock<const Eigen::VectorXd, -1>
    Model::jointLimitsLower() const
{
    return _limitPosLower.segment(6, sizeJoint());
}
Eigen::VectorBlock<const Eigen::VectorXd, -1>
    Model::jointLimitsUpper() const
{
    return _limitPosUpper.segment(6, sizeJoint());
}
Eigen::VectorBlock<const Eigen::VectorXd, -1>
    Model::jointLimitsVelocity() const
{
    return _limitVelocity.segment(6, sizeJoint());
}
Eigen::VectorBlock<const Eigen::VectorXd, -1>
    Model::jointLimitsTorque() const
{
    return _limitEffort.segment(6, sizeJoint());
}

Eigen::Vector3d Model::position(
    size_t srcFrameId, 
    size_t dstFrameId,
    const Eigen::Vector3d& point) const
{
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::position: "
            "Internal model is not updated.");
    }
    //No transformation if no actual frame change
    if (srcFrameId == dstFrameId) {
        return point;
    }

    //Compute transformation from body src to base 
    //and then from base to body dst
    RBDLMath::Vector3d ptInBase;
    if (srcFrameId != 0) {
        //Since model update is explicitly disabled, 
        //the model will be const in RBDL implementation
        ptInBase = RBDL::CalcBodyToBaseCoordinates(
            const_cast<RBDL::Model&>(_model), 
            _position, srcFrameId, point, false);
    } else {
        //No transformation if src frame 
        //is already the root
        ptInBase = point;
    }
    RBDLMath::Vector3d ptInBody;
    if (dstFrameId != 0) {
        //Since model update is explicitly disabled, 
        //the model will be const in RBDL implementation
        ptInBody = RBDL::CalcBaseToBodyCoordinates(
            const_cast<RBDL::Model&>(_model), 
            _position, dstFrameId, ptInBase, false);
    } else {
        //No transformation if dst frame 
        //is already the root
        ptInBody = ptInBase;
    }

    return ptInBody;
}
Eigen::Vector3d Model::position(
    const std::string& srcFrameName, 
    const std::string& dstFrameName,
    const Eigen::Vector3d& point) const
{
    return position(
        getIndexFrame(srcFrameName), 
        getIndexFrame(dstFrameName),
        point);
}

Eigen::Matrix3d Model::orientation(
    size_t srcFrameId, 
    size_t dstFrameId) const
{
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::orientation: "
            "Internal model is not updated.");
    }
    //No transformation if no actual frame change
    if (srcFrameId == dstFrameId) {
        return Eigen::Matrix3d::Identity();
    }

    //Compute transformation from body src to base 
    //and then from base to body dst
    RBDLMath::Matrix3d transformSrcToBase;
    if (srcFrameId != 0) {
        //Since model update is explicitly disabled, 
        //the model will be const in RBDL implementation
        transformSrcToBase = RBDL::CalcBodyWorldOrientation(
            const_cast<RBDL::Model&>(_model), 
            _position, srcFrameId, false);
    } else {
        //No transformation if src frame 
        //is already the root
        transformSrcToBase = Eigen::Matrix3d::Identity();
    }
    RBDLMath::Matrix3d transformDstToBase;
    if (dstFrameId != 0) {
        //Since model update is explicitly disabled, 
        //the model will be const in RBDL implementation
        transformDstToBase = RBDL::CalcBodyWorldOrientation(
            const_cast<RBDL::Model&>(_model), 
            _position, dstFrameId, false);
    } else {
        //No transformation if dst frame 
        //is already the root
        transformDstToBase = Eigen::Matrix3d::Identity();
    }

    return transformDstToBase*transformSrcToBase.transpose();
}
Eigen::Matrix3d Model::orientation(
    const std::string& srcFrameName, 
    const std::string& dstFrameName) const
{
    return orientation(
        getIndexFrame(srcFrameName), 
        getIndexFrame(dstFrameName));
}

double Model::orientationYaw(
    size_t srcFrameId, 
    size_t dstFrameId) const
{
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::orientationYaw: "
            "Internal model is not updated.");
    }
    //No transformation if no actual frame change
    if (srcFrameId == dstFrameId) {
        return 0.0;
    }

    Eigen::Matrix3d mat = orientation(srcFrameId, dstFrameId);
    return std::atan2(mat(1,0), mat(0,0));
}
double Model::orientationYaw(
    const std::string& srcFrameName, 
    const std::string& dstFrameName) const
{
    return orientationYaw(
        getIndexFrame(srcFrameName),
        getIndexFrame(dstFrameName));
}

Eigen::Matrix6d Model::getWrenchTransform(
    const Eigen::Vector3d& posDstInSrc,
    const Eigen::Matrix3d& matDstInSrc) const
{
    Eigen::Matrix6d transform = Eigen::Matrix6d::Zero();
    Eigen::Matrix3d matSrcInDst = matDstInSrc.transpose();

    transform.block(0,0,3,3) = matSrcInDst;
    transform.block(0,3,3,3) = -matSrcInDst*SkewMatrix(posDstInSrc);
    transform.block(3,3,3,3) = matSrcInDst;
    
    return transform;
}

Eigen::Vector6d Model::wrenchTransform(
    size_t srcFrameId, 
    size_t dstFrameId,
    const Eigen::Vector6d& wrenchInSrc) const
{
    Eigen::Vector3d posDstInSrc =
        position(dstFrameId, srcFrameId);
    Eigen::Matrix3d matSrcInDst = 
        orientation(srcFrameId, dstFrameId); 

    Eigen::Vector6d wrenchInDst;
    wrenchInDst.segment(0, 3) = matSrcInDst*(
        wrenchInSrc.segment(0, 3) -
        posDstInSrc.cross(wrenchInSrc.segment<3>(3, 3)));
    wrenchInDst.segment(3, 3) = 
        matSrcInDst*wrenchInSrc.segment(3, 3);

    return wrenchInDst;
}
Eigen::Vector6d Model::wrenchTransform(
    const std::string& srcFrameName, 
    const std::string& dstFrameName,
    const Eigen::Vector6d& wrench) const
{
    return wrenchTransform(
        getIndexFrame(srcFrameName),
        getIndexFrame(dstFrameName),
        wrench);
}

void Model::setBaseToMatchFramePosition(
    size_t frameId, 
    const Eigen::Vector3d& posInWorld)
{
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::setBaseToMatchFramePosition: "
            "Internal model is not updated.");
    }
    
    //Compute relative pose from floating 
    //base root to frame
    Eigen::Vector3d baseToFramePosInBase = 
        position(frameId, _baseFrameId);
    Eigen::Vector3d baseToFramePosInWorld = 
        orientation(_baseFrameId, 0)*baseToFramePosInBase;
    //Compute new base position to match expected position
    Eigen::Vector3d newBasePos = 
        posInWorld - baseToFramePosInWorld;
    //Assign floating base
    setBasePos(newBasePos);
    
    //Set model state as dirty
    _isDirty = true;
}
void Model::setBaseToMatchFramePosition(
    const std::string& frameName, 
    const Eigen::Vector3d& posInWorld)
{
    setBaseToMatchFramePosition(
        getIndexFrame(frameName),
        posInWorld);
}
        
void Model::setBaseToMatchFrameOrientation(
    size_t frameId, 
    const Eigen::Quaterniond& quatInWorld)
{
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::setBaseToMatchFrameOrientation: "
            "Internal model is not updated.");
    }

    //Compute relative orientation from 
    //floating base root to frame
    Eigen::Matrix3d baseToFrameMat = 
        orientation(frameId, _baseFrameId);
    //Compute new base orientation to 
    //match given expected pose
    Eigen::Quaterniond newBaseQuat = 
        quatInWorld*Eigen::Quaterniond(baseToFrameMat).inverse();
    //Renormalize quaternion because of numerical errors
    newBaseQuat.normalize();
    //Assign floating base
    setBaseQuat(newBaseQuat);

    //Set model state as dirty
    _isDirty = true;
}
void Model::setBaseToMatchFrameOrientation(
    const std::string& frameName, 
    const Eigen::Quaterniond& quatInWorld)
{
    setBaseToMatchFrameOrientation(
        getIndexFrame(frameName),
        quatInWorld);
}

void Model::setBaseToMatchFramePose(
    size_t frameId, 
    const Eigen::Vector3d& pointInFrame,
    const Eigen::Vector3d& posInWorld,
    const Eigen::Quaterniond& quatInWorld)
{
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::setBaseToMatchFramePose: "
            "Internal model is not updated.");
    }

    //Compute relative pose from floating 
    //base root to frame
    Eigen::Vector3d baseToFramePos = 
        position(frameId, _baseFrameId, pointInFrame);
    Eigen::Matrix3d baseToFrameMat = 
        orientation(frameId, _baseFrameId);
    //Compute new base pose to match given expected pose
    Eigen::Vector3d baseToFramePosInWorld = 
        quatInWorld.toRotationMatrix()
        * baseToFrameMat.transpose()
        * baseToFramePos;
    Eigen::Vector3d newBasePos = 
        posInWorld - baseToFramePosInWorld;
    Eigen::Quaterniond newBaseQuat = 
        quatInWorld*Eigen::Quaterniond(baseToFrameMat).inverse();
    //Renormalize quaternion because of numerical errors
    newBaseQuat.normalize();
    //Assign floating base
    setBasePos(newBasePos);
    setBaseQuat(newBaseQuat);

    //Set model state as dirty
    _isDirty = true;
}
void Model::setBaseToMatchFramePose(
    const std::string& frameName, 
    const Eigen::Vector3d& pointInFrame,
    const Eigen::Vector3d& posInWorld,
    const Eigen::Quaterniond& quatInWorld)
{
    setBaseToMatchFramePose(
        getIndexFrame(frameName),
        pointInFrame,
        posInWorld,
        quatInWorld);
}

void Model::setBaseVeltoMatchFrameVel(
    size_t frameId, 
    const Eigen::Vector6d& velocityInWorld)
{
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::setBaseVeltoMatchFrameVel: "
            "Internal model is not updated.");
    }

    //Split DOF velocities and Jacobian 
    //between base and joints
    size_t size = sizeJoint();
    Eigen::VectorXd velJoint = _velocity.segment(6, size);
    Eigen::MatrixXd jac = pointJacobian(frameId, 0);
    Eigen::MatrixXd jacBase = jac.block(0, 0, 6, 6);
    Eigen::MatrixXd jacJoint = jac.block(0, 6, 6, size);
    //Compute the new base velocity
    Eigen::VectorXd velBase =
        jacBase.householderQr().solve(velocityInWorld - jacJoint*velJoint);
    //Assign it to state
    _velocity.segment(0, 6) = velBase;
    
    //Set model state as dirty
    _isDirty = true;
}
void Model::setBaseVeltoMatchFrameVel(
    const std::string& frameName, 
    const Eigen::Vector6d& velocityInWorld)
{
    setBaseVeltoMatchFrameVel(
        getIndexFrame(frameName),
        velocityInWorld);
}
        
Eigen::Vector3d Model::centerOfMass(
    size_t dstFrameId) const
{
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::centerOfMass: "
            "Internal model is not updated.");
    }

    //Compute CoM
    double mass;
    RBDLMath::Vector3d com;
    RBDL::Utils::CalcCenterOfMass(
        const_cast<RBDL::Model&>(_model), 
        _position, 
        Eigen::VectorXd::Zero(sizeVectVel()),
        NULL, mass, com, NULL, NULL, NULL, NULL, false);

    //Convert in destination frame if needed
    if (dstFrameId != 0) {
        com = position(0, dstFrameId, com);
    }

    return com;
}
Eigen::Vector3d Model::centerOfMass(
    const std::string& dstFrameName) const
{
    return centerOfMass(
        getIndexFrame(dstFrameName));
}
        
double Model::massSum() const
{
    double mass;
    RBDLMath::Vector3d com;
    RBDL::Utils::CalcCenterOfMass(
        const_cast<RBDL::Model&>(_model), 
        _position, 
        Eigen::VectorXd::Zero(sizeVectVel()),
        NULL, mass, com, NULL, NULL, NULL, NULL, false);

    return mass;
}
        
void Model::setGravity(const Eigen::Vector3d& vect)
{
    _model.gravity = vect;
}

Eigen::Vector6d Model::pointVelocity(
    size_t frameSrcId,
    size_t frameDstId,
    const Eigen::Vector3d& point)
{
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::pointVelocity: "
            "Internal model is not updated.");
    }

    //Compute velocity in world frame
    Eigen::Vector6d vel = RBDL::CalcPointVelocity6D(
        _model, _position, _velocity, 
        frameSrcId, point, false);

    //Conversion in destination frame
    if (frameDstId != 0) {
        Eigen::Matrix3d mat = orientation(0, frameDstId);
        Eigen::Vector3d rot = vel.segment(0, 3);
        Eigen::Vector3d trans = vel.segment(3, 3);
        vel.segment(0, 3) = mat * rot;
        vel.segment(3, 3) = mat * trans;
    }

    return vel;
}
Eigen::Vector6d Model::pointVelocity(
    const std::string& frameSrcName, 
    const std::string& frameDstName, 
    const Eigen::Vector3d& point)
{
    return pointVelocity(
        getIndexFrame(frameSrcName), 
        getIndexFrame(frameDstName), 
        point);
}

Eigen::Vector6d Model::pointAcceleration(
    size_t frameSrcId,
    size_t frameDstId,
    const Eigen::VectorXd& acceleration,
    const Eigen::Vector3d& point)
{
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::pointAcceleration: "
            "Internal model is not updated.");
    }
    //Check vector size
    if ((size_t)acceleration.size() != sizeVectVel()) {
        throw std::logic_error(
            "inria::Model::pointAcceleration: "
            "Invalid acceleration vector size: "
            + std::to_string(acceleration.size()));
    }

    //Compute acceleration in world frame.
    //WARNING: to be sure that kinematics acceleration
    //is correct, the flag for kinematics update is
    //set to true
    Eigen::Vector6d acc = RBDL::CalcPointAcceleration6D(
        _model, _position, _velocity, acceleration,
        frameSrcId, point, true);

    //Conversion in destination frame
    if (frameDstId != 0) {
        Eigen::Matrix3d mat = orientation(0, frameDstId);
        Eigen::Vector3d rot = acc.segment(0, 3);
        Eigen::Vector3d trans = acc.segment(3, 3);
        acc.segment(0, 3) = mat * rot;
        acc.segment(3, 3) = mat * trans;
    }

    return acc;
}
Eigen::Vector6d Model::pointAcceleration(
    const std::string& frameSrcName, 
    const std::string& frameDstName, 
    const Eigen::VectorXd& acceleration,
    const Eigen::Vector3d& point)
{
    return pointAcceleration(
        getIndexFrame(frameSrcName), 
        getIndexFrame(frameDstName), 
        acceleration,
        point);
}

Eigen::MatrixXd Model::pointJacobian(
    size_t frameSrcId,
    size_t frameDstId,
    const Eigen::Vector3d& point) const
{
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::pointJacobian: "
            "Internal model is not updated.");
    }

    //Init matrix
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, sizeVectVel());

    if (frameSrcId == frameDstId) {
        //If the source and destination frame are the same, 
        //we are actually asking for the body spatial Jacobian.
        //RBDL implementation is used for (very) slight 
        //performance gain (back conversion is not needed).
        RBDL::CalcBodySpatialJacobian(
            const_cast<RBDL::Model&>(_model), 
            _position, frameSrcId, J, false);
    } else if (frameDstId == 0) {
        //Compute Jacobian on given point 
        //in world frame
        RBDL::CalcPointJacobian6D(
            const_cast<RBDL::Model&>(_model), 
            _position, frameSrcId, point, J, false);
    } else {
        //General case with relative Jacobian implementation.
        //See paper:
        //A More Compact Expression of Relative Jacobian
        //Based on Individual Manipulator Jacobians
        //By Rodrigo S. Jamisola, Rodney G. Roberts
        Eigen::MatrixXd srcJ = Eigen::MatrixXd::Zero(6, sizeVectVel());
        Eigen::MatrixXd dstJ = Eigen::MatrixXd::Zero(6, sizeVectVel());
        RBDL::CalcBodySpatialJacobian(
            const_cast<RBDL::Model&>(_model), 
            _position, frameSrcId, srcJ, false);
        RBDL::CalcBodySpatialJacobian(
            const_cast<RBDL::Model&>(_model), 
            Eigen::Vector3d::Zero(), frameDstId, dstJ, false);
        Eigen::Vector3d pos = position(frameSrcId, frameDstId);
        Eigen::Matrix3d mat = orientation(frameSrcId, frameDstId);
        Eigen::Matrix3d skew = SkewMatrix(-pos);
        J.block(0,0,3,sizeVectVel()) = 
            mat*srcJ.block(0,0,3,sizeVectVel())
            - dstJ.block(0,0,3,sizeVectVel());
        J.block(3,0,3,sizeVectVel()) = 
            mat*srcJ.block(3,0,3,sizeVectVel())
            - dstJ.block(3,0,3,sizeVectVel())
            - skew*dstJ.block(0,0,3,sizeVectVel());
    }
    
    return J;
}
Eigen::MatrixXd Model::pointJacobian(
    const std::string& frameSrcName,
    const std::string& frameDstName,
    const Eigen::Vector3d& point) const
{
    return pointJacobian(
        getIndexFrame(frameSrcName), 
        getIndexFrame(frameDstName), 
        point);
}
        
Eigen::MatrixXd Model::comJacobian(
    size_t frameDstId) const
{
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::comJacobian: "
            "Internal model is not updated.");
    }
    
    //Init temporary and accumulation 
    //matrices and mass
    double sumMass = 0.0;
    Eigen::MatrixXd tmpJ = Eigen::MatrixXd::Zero(3, sizeVectVel());
    Eigen::MatrixXd sumJ = Eigen::MatrixXd::Zero(3, sizeVectVel());
    //Compute weighted average of all Jacobian 
    //matrices on each body center of mass
    for (size_t i=1;i<_model.mBodies.size();i++) {
        //Current body mass
        double mass = _model.mBodies[i].mMass;
        if (mass > 0.0 && !_model.mBodies[i].mIsVirtual) {
            //Current body center of mass in body frame
            const Eigen::Vector3d& center = 
                _model.mBodies[i].mCenterOfMass;
            //Compute the Jacobian matrix of current body
            //on local center of mass position in world frame
            tmpJ.setZero();
            RBDL::CalcPointJacobian(
                const_cast<RBDL::Model&>(_model), 
                _position, i, center, tmpJ, false);
            //Apply mass weighting and sum the Jacobian
            sumMass += mass;
            tmpJ *= mass;
            sumJ += tmpJ;
        }
    }
    //Normalize the accumulated Jacobian
    if (sumMass > 0.0) {
        sumJ *= (1.0/sumMass);
    }
    
    //Conversion of the matrix to dst 
    //frame only if needed
    if (frameDstId != 0) {
        Eigen::Matrix3d mat = orientation(0, frameDstId);
        Eigen::Vector3d trans;
        for (size_t i=0;i<(size_t)sumJ.cols();i++) {
            trans = sumJ.block(0, i, 3, 1);
            sumJ.block(0, i, 3, 1) = mat * trans;
        }
    }

    return sumJ;
}
Eigen::MatrixXd Model::comJacobian(
    const std::string& frameDstName) const
{
    return comJacobian(
        getIndexFrame(frameDstName));
}
        
Eigen::VectorXd Model::computeGravityVector()
{
    size_t size = sizeVectVel();
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::computeGravityVector: "
            "Internal model is not updated.");
    }
    
    //WARNING: RBDL NonlinearEffects dynamics function
    //update internal model velocity state.
    //To enforce correctness to following kinematics
    //function, velocity state must be reset afterward.
    Eigen::VectorXd G = Eigen::VectorXd::Zero(size);
    RBDL::NonlinearEffects(
        _model, _position, Eigen::VectorXd::Zero(size), G);
    //Reset internal RBDL velocity
    updateState();

    return G;
}

void Model::computeEquationOfMotion(
    Eigen::MatrixXd& H,
    Eigen::VectorXd& C,
    Eigen::VectorXd* G)
{
    size_t size = sizeVectVel();
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::computeEquationOfMotion: "
            "Internal model is not updated.");
    }
    
    //WARNING: RBDL NonlinearEffects dynamics function
    //update internal model velocity state.
    //To enforce correctness to following kinematics
    //function, the gravity term (zero velocity)
    //must be computed first !!!
    if (G != nullptr) {
        *G = Eigen::VectorXd::Zero(size);
        RBDL::NonlinearEffects(
            _model, _position, Eigen::VectorXd::Zero(size), *G);
    }

    C = Eigen::VectorXd::Zero(size);
    RBDL::NonlinearEffects(
        _model, _position, _velocity, C);
    H = Eigen::MatrixXd::Zero(size, size);
    RBDL::CompositeRigidBodyAlgorithm(
        _model, _position, H, false);
}

Eigen::VectorXd Model::computeJointTorqueSingleContact(
    size_t frameId,
    Eigen::Vector6d* wrench)
{
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::computeJointTorqueSingleContact: "
            "Internal model is not updated.");
    }
    
    size_t size = sizeJoint();
    Eigen::VectorXd grav = computeGravityVector();
    Eigen::VectorXd gravBase = grav.segment(0, 6);
    Eigen::VectorXd gravJoint = grav.segment(6, size);
    Eigen::MatrixXd jac = pointJacobian(frameId, frameId);
    Eigen::MatrixXd jacBase = jac.block(0, 0, 6, 6);
    Eigen::MatrixXd jacJoint = jac.block(0, 6, 6, size);
    Eigen::VectorXd lambda = 
        jacBase.transpose().colPivHouseholderQr().solve(gravBase);
    if (wrench != nullptr) {
        *wrench = lambda;
    }
    return gravJoint - jacJoint.transpose()*lambda;
}
Eigen::VectorXd Model::computeJointTorqueSingleContact(
    const std::string& frameName,
    Eigen::Vector6d* wrench)
{
    return computeJointTorqueSingleContact(
        getIndexFrame(frameName), wrench);
}

void Model::computeCentroidalDynamics(
    const Eigen::MatrixXd& H,
    const Eigen::VectorXd& C,
    const Eigen::VectorXd& G,
    Eigen::MatrixXd& CMM,
    Eigen::VectorXd& CMMDotQDot)
{
    size_t size = sizeVectVel();
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::computeCentroidalDynamics: "
            "Internal model is not updated.");
    }
    //Check vector sizes
    if (
        (size_t)C.size() != size ||
        (size_t)H.rows() != size ||
        (size_t)H.cols() != size
    ) {
        throw std::logic_error(
            "inria::Model::computeCentroidalDynamics: "
            "Invalid input matrix or vector size.");
    }

    //Implementation written by Iordanis Chatzinikolaidis
    //following the notations from:
    //"Improved Computation of the Humanoid Centroidal
    //Dynamics and Application for Whole-Body Control",
    //Wensing, Orin, 2016
    
    //Compute the generalized joint-space 
    //forces without the gravitational component 
    //(only centrifugal and Coriolis).
    //Remove gravity from others nonlinear term.
    Eigen::VectorXd N = C - G;

    //WARNING: 
    //RBDL default floating base DOF order is:
    //(translation, rotation).
    //But DOF velocity (the same for accelerations)
    //are stored in following convention:
    //- base linear 3d velocity with respect 
    //to origin expressed in word frame
    //- base angular 3d velocity with respect 
    //to origin expressed in BODY FRAME (floating base)
    //- joint angular velocities

    //Build a block rotation matrix transforming
    //the angular velocity from body to world frame
    //as well as swapping the angular and translation
    //from (trans, angular) to (angular, trans)
    //matching the RBDL jacobian6D convention.
    //
    //The following Psi matrix is actually the inverse 
    //of that (phi) transformation matrix (see paper).
    Eigen::Matrix3d b_E_w = orientation(_baseFrameId, 0);
    Eigen::MatrixXd Psi_1 = Eigen::MatrixXd::Zero(6,6);
    Psi_1.block(3, 0, 3, 3) = Eigen::Matrix3d::Identity();
    Psi_1.block(0, 3, 3, 3) = b_E_w;

    //Compute the locked inertial matrix (associated 
    //with floating base) expressed in floating base frame
    Eigen::MatrixXd I_C_1 = Psi_1.transpose()*H.block(0, 0, 6, 6)*Psi_1;

    //Extract the total system mass from it
    double mass = I_C_1(5,5);

    //Also extract the position of the center of mass 
    //expressed in floating base body frame
    Eigen::Vector3d b_p_G = 
        1.0/mass*Eigen::Vector3d(
            I_C_1(2, 4), I_C_1(0, 5), I_C_1(1,3));
    //Compute the spatial transformation from floating 
    //base frame to center of mass frame
    RBDLMath::SpatialTransform G_X_b = 
        RBDLMath::SpatialTransform(b_E_w, b_p_G);

    //Finally compute and assign 
    //the centroidal momentum matrix
    CMM = G_X_b.toMatrixAdjoint()*Psi_1.transpose()*H.block(0, 0, 6, size);
    //As well as the bias term
    CMMDotQDot = G_X_b.toMatrixAdjoint()*Psi_1.transpose()*N.segment(0,6);
}

Eigen::VectorXd Model::inverseDynamics(
    const Eigen::VectorXd& acceleration,
    const std::map<std::string, Eigen::Vector6d>& externalWrenches)
{
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::inverseDynamics: "
            "Internal model is not updated.");
    }

    //Check vector sizes
    if ((size_t)acceleration.size() != sizeVectVel()) {
        throw std::logic_error(
            "inria::Model::inverseDynamics: "
            "Invalid acceleration vector size: "
            + std::to_string(acceleration.size()));
    }
    
    //Initialize the vector of external forces
    //applied on every links and expressed in 
    //world frame coordinates
    std::vector<RBDLMath::SpatialVector> externalForcesInBase;
    if (externalWrenches.size() > 0) {
        for (size_t i=0;i<_model.mBodies.size();i++) {
            externalForcesInBase.push_back(Eigen::Vector6d::Zero());
        }
        //Add the given external wrenches into container
        //and convert them from local frame to world frame
        for (const auto& it : externalWrenches) {
            size_t frameId = getIndexFrame(it.first);
            //Retrieve the spatial transformation
            RBDLMath::SpatialTransform baseToBody;
            if (_model.IsFixedBodyId(frameId)) {
                //Fixed body case. Retrieve parent movable body
                //id and transformation matrix.
                unsigned int fbodyId = frameId - _model.fixed_body_discriminator;
                //Transformation from base to frameFixedId
                frameId = _model.mFixedBodies[fbodyId].mMovableParent;
                baseToBody = _model.mFixedBodies[fbodyId].mParentTransform 
                    * _model.X_base[frameId];
            } else {
                baseToBody = _model.X_base[frameId];
            }
            //Apply the transformation
            externalForcesInBase.at(frameId) = 
                baseToBody.inverse().toMatrixAdjoint() * it.second;
        }
    }

    RBDLMath::VectorNd tau(sizeVectVel());
    tau.setZero();
    if (externalWrenches.size() > 0) {
        RBDL::InverseDynamics(
            _model,
            _position, _velocity, acceleration,
            tau, &externalForcesInBase);
    } else {
        RBDL::InverseDynamics(
            _model,
            _position, _velocity, acceleration,
            tau, nullptr);
    }

    return tau;
}

Eigen::VectorXd Model::inverseDynamicsContact(
    size_t frameFixedId,
    const Eigen::VectorXd& acceleration,
    Eigen::Vector6d* contactWrench)
{
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::inverseDynamics: "
            "Internal model is not updated.");
    }

    //Check vector sizes
    if ((size_t)acceleration.size() != sizeVectVel()) {
        throw std::logic_error(
            "inria::Model::inverseDynamics: "
            "Invalid acceleration vector size: "
            + std::to_string(acceleration.size()));
    }

    unsigned int usedDOFCount = sizeVectVel() - 6;
    
    //Compute spatial Jacobian for fixed body
    //See Featherstone's book "Rigid Body Dynamics Algorithms"
    //Chapter 8.2 page 145
    RBDLMath::MatrixNd allK(6, sizeVectVel());
    allK.setZero();
    RBDL::CalcBodySpatialJacobian(
        _model, _position, frameFixedId, allK, false);

    //Trim constraints to remove floating base DOF
    RBDLMath::MatrixNd K = allK.rightCols(usedDOFCount);

    //Compute K kernel basis G such as K*G = 0
    Eigen::FullPivLU<RBDLMath::MatrixNd> lu(K);
    RBDLMath::MatrixNd G = lu.kernel();

    //Compute Inverse Dynamics for kinematics tree model
    //without loop and constraint
    RBDLMath::VectorNd allTauID(sizeVectVel());
    allTauID.setZero();
    RBDL::InverseDynamics(
        _model, 
        _position, _velocity, acceleration,
        allTauID, nullptr);
    
    //Trim resulting torques to real used DOF as InverseDynamics
    //torques does only depend on children bodies
    //(floating base in single support does not change 
    //InverseDynamics result)
    RBDLMath::VectorNd tauID = allTauID.tail(usedDOFCount);

    //Compute right side of equation to solve
    //G' * tau = G' * tauID = tauIDGt
    RBDLMath::VectorNd tauIDGt = G.transpose()*tauID;

    //Build SVD decomposition of G' in order to retrieve it
    //pseudo inverse.
    Eigen::JacobiSVD<RBDLMath::MatrixNd> svd(G.transpose(), 
        Eigen::ComputeThinU | Eigen::ComputeThinV);
    //Solve the equation G' * tau = tauIDGt with least square 
    //since the equation are underdetermined (infinite solution) 
    //and we are looking for smallest possible torques configuration
    //Base DOFs are trimmed in order to not take them into 
    //account in torque L2 minimization.
    RBDLMath::VectorNd resultTau = svd.solve(tauIDGt);

    //The given frameFixedId is possible a fixed body
    //(id >= max_int/2). Retrieve parent movable body
    //id and transformation matrix.
    //Used movable body id
    unsigned int referenceBodyId = frameFixedId;
    //Transformation from base to frameFixedId
    RBDLMath::SpatialTransform baseToBody;
    if (_model.IsFixedBodyId(frameFixedId)) {
        unsigned int fbodyId = frameFixedId - _model.fixed_body_discriminator;
        referenceBodyId = _model.mFixedBodies[fbodyId].mMovableParent;
        baseToBody = _model.mFixedBodies[fbodyId].mParentTransform 
            * _model.X_base[referenceBodyId];
    } else {
        baseToBody = _model.X_base[referenceBodyId];
    }

    //Compute the Cartesian constraints force 6D lambda 
    //(moments, linear forces) in frameFixedId frame.
    //resultTau = (H.ddq + C - tauA) - K'.lambda
    //=> Solve K'.lambda = tauID - resultTau
    //Lambda is expressed in frameFixedId frame.
    Eigen::VectorXd lambda = 
        K.transpose().colPivHouseholderQr().solve(tauID - resultTau);
    if (contactWrench != nullptr) {
        *contactWrench = lambda;
    }
    
    //Now, we need to recompute an InverseDynamics to compute
    //the base DOFs that we trimmed for pseudo inverse L2 minimization.
    //Single support InverseDynamics is called with contact forces added
    //on frameFixedId in order to compute allResultTau for double support.
    std::vector<RBDLMath::SpatialVector> externalForcesInBase;
    for (size_t i=0;i<_model.mBodies.size();i++) {
        externalForcesInBase.push_back(Eigen::Vector6d::Zero());
    }
    //External forces are expressed in base frame.
    //Lambda is in frameFixedId frame. So conversion is needed.
    //(moments are dependent of the reference point).
    externalForcesInBase[referenceBodyId] = 
        baseToBody.inverse().toMatrixAdjoint() * lambda;
    //Compute InverseDynamics
    RBDLMath::VectorNd allResultTau(sizeVectVel());
    allResultTau.setZero();
    RBDL::InverseDynamics(
        _model, 
        _position, _velocity, acceleration,
        allResultTau, &externalForcesInBase);

    return allResultTau;
}
Eigen::VectorXd Model::inverseDynamicsContact(
    const std::string& frameFixedName,
    const Eigen::VectorXd& acceleration,
    Eigen::Vector6d* contactWrench)
{
    return inverseDynamicsContact(
        getIndexFrame(frameFixedName),
        acceleration, contactWrench);
}

Eigen::VectorXd Model::forwardDynamicsContacts(
    RBDL::ConstraintSet& constraints, 
    const Eigen::VectorXd& position,
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& torque)
{
    //Sanity check
    if ((size_t)position.size() != sizeVectPos()) {
        throw std::logic_error(
            "inria::Model::forwardDynamicsContacts: "
            "Invalid position vector size.");
    }
    if ((size_t)velocity.size() != sizeVectVel()) {
        throw std::logic_error(
            "inria::Model::forwardDynamicsContacts: "
            "Invalid velocity vector size.");
    }
    if ((size_t)torque.size() != sizeVectVel()) {
        throw std::logic_error(
            "inria::Model::forwardDynamicsContacts: "
            "Invalid torque vector size.");
    }

    RBDLMath::VectorNd QDDot(sizeVectVel());
    RBDL::ForwardDynamicsConstraintsDirect(
        _model, 
        position, velocity, torque, 
        constraints, QDDot, NULL);

    //RBDL Dynamics functions update internal
    //model state. An updateState() is required to
    //enforce correctness of kinematics functions.
    _isDirty = true;

    return QDDot;
}

Eigen::VectorXd Model::forwardImpulsiveDynamicsContacts(
    double dt, 
    RBDL::ConstraintSet& constraints, 
    const Eigen::VectorXd& position,
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& torque)
{
    //Sanity check
    if ((size_t)position.size() != sizeVectPos()) {
        throw std::logic_error(
            "inria::Model::forwardImpulsiveDynamicsContacts: "
            "Invalid position vector size.");
    }
    if ((size_t)velocity.size() != sizeVectVel()) {
        throw std::logic_error(
            "inria::Model::forwardImpulsiveDynamicsContacts: "
            "Invalid velocity vector size.");
    }
    if ((size_t)torque.size() != sizeVectVel()) {
        throw std::logic_error(
            "inria::Model::forwardImpulsiveDynamicsContacts: "
            "Invalid torque vector size.");
    }
    
    //Retrieve sizes
    size_t sizeDOF = sizeVectVel();
    size_t sizeCst = constraints.size();
    
    //Compute full H, K matrix and C
    //vectors into the constraint set
    RBDL::CalcConstrainedSystemVariables(
        _model, position, velocity, torque, constraints);
    
    //RBDL Dynamics functions update internal
    //model state. An updateState() is required to
    //enforce correctness of kinematics functions.
    _isDirty = true;

    //Build matrix system
    constraints.A.setZero();
    constraints.b.setZero();
    constraints.A.block(0, 0, sizeDOF, sizeDOF) = 
        constraints.H;
    constraints.A.block(sizeDOF, 0, sizeCst, sizeDOF) = 
        constraints.G;
    constraints.A.block(0, sizeDOF, sizeDOF, sizeCst) = 
        dt*constraints.G.transpose();
    constraints.b.segment(0, sizeDOF) = 
        dt*(torque - constraints.C) + constraints.H*velocity;
    
    //Solve the linear system
    constraints.x = constraints.
        A.fullPivHouseholderQr().solve(constraints.b);
            
    //Copy computed force
    constraints.force = -constraints.x.segment(sizeDOF, sizeCst);
    //Return computed next velocity
    return constraints.x.segment(0, sizeDOF);
}

Eigen::VectorXd Model::impulseContacts(
    RBDL::ConstraintSet& constraints,
    const Eigen::VectorXd& position,
    const Eigen::VectorXd& velocity)
{
    //Sanity check
    if ((size_t)position.size() != sizeVectPos()) {
        throw std::logic_error(
            "inria::Model::impulseContacts: "
            "Invalid position vector size.");
    }
    if ((size_t)velocity.size() != sizeVectVel()) {
        throw std::logic_error(
            "inria::Model::impulseContacts: "
            "Invalid velocity vector size.");
    }

    Eigen::VectorXd newVel = velocity;
    RBDL::ComputeConstraintImpulsesDirect(
        _model, position, velocity, constraints, newVel);
    
    //RBDL Dynamics functions update internal
    //model state. An updateState() is required to
    //enforce correctness of kinematics functions.
    _isDirty = true;

    return newVel;
}

double Model::getBodyMass(size_t frameId) const
{
    //Fixed body case. 
    //Retrieve parent movable body id.
    unsigned int movableId = frameId;
    if (
        frameId != 0 && 
        const_cast<RBDL::Model&>(_model).IsFixedBodyId(frameId)
    ) {
        unsigned int fixedId = frameId - _model.fixed_body_discriminator;
        movableId = _model.mFixedBodies[fixedId].mMovableParent;
    } 

    return _model.mBodies[movableId].mMass;
}
double Model::getBodyMass(const std::string& frameName) const
{
    return getBodyMass(getIndexFrame(frameName));
}
Eigen::Vector3d Model::getBodyCoM(size_t frameId) const
{
    //Fixed body case. 
    //Retrieve parent movable body id.
    unsigned int movableId = frameId;
    if (
        frameId != 0 && 
        const_cast<RBDL::Model&>(_model).IsFixedBodyId(frameId)
    ) {
        unsigned int fixedId = frameId - _model.fixed_body_discriminator;
        movableId = _model.mFixedBodies[fixedId].mMovableParent;
    } 

    //Center of mass is expressed in movanle body parent frame. 
    //Transformation back to given frame.
    return position(
        movableId, frameId,
        _model.mBodies[movableId].mCenterOfMass);
}
Eigen::Vector3d Model::getBodyCoM(const std::string& frameName) const
{
    return getBodyCoM(getIndexFrame(frameName));
}

double Model::energyPotential() const
{
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::energyPotential: "
            "Internal model is not updated.");
    }
    
    //Retrieve gravity constant
    double g = _model.gravity.norm();
    //Iterate over all body masses
    double energy = 0.0;
    for (size_t i=0;i<_model.mBodies.size();i++) {
        double mass = _model.mBodies[i].mMass;
        if (mass > 0.0 && !_model.mBodies[i].mIsVirtual) {
            //Current body center of mass in body frame
            const Eigen::Vector3d& centerInBody = 
                _model.mBodies[i].mCenterOfMass;
            //Conversion in world frame
            Eigen::Vector3d centerInWorld = position(i, 0, centerInBody);
            //Sum up energy
            energy += mass*g*centerInWorld.z();
        }
    }

    return energy;
}
double Model::energyKinetic() const
{
    //Check if internal RBDL model is updated
    if (_isDirty) {
        throw std::logic_error(
            "inria::Model::energyKinetic: "
            "Internal model is not updated.");
    }
    
    //Iterate over all body masses
    double energy = 0.0;
    for (size_t i=0;i<_model.mBodies.size();i++) {
        double mass = _model.mBodies[i].mMass;
        if (mass > 0.0 && !_model.mBodies[i].mIsVirtual) {
            //Current body center of mass in body frame
            const Eigen::Vector3d& centerInBody = 
                _model.mBodies[i].mCenterOfMass;
            //Current body center of mass velocity
            Eigen::Vector3d centerVel = RBDL::CalcPointVelocity(
                const_cast<RBDL::Model&>(_model),
                _position, _velocity, i, 
                centerInBody, false);
            //Sum up energy
            energy += 0.5*mass*centerVel.squaredNorm();
        }
    }

    return energy;
}

const RBDL::Model& Model::getRBDLModel() const
{
    return _model;
}
RBDL::Model& Model::getRBDLModel()
{
    return _model;
}
        
const std::string& Model::getPathURDF() const
{
    return _pathURDF;
}

}

