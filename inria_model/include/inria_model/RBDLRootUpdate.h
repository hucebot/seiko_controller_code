#ifndef INRIA_MODEL_RBDLROOTUPDATE_H
#define INRIA_MODEL_RBDLROOTUPDATE_H

#include <vector>
#include <rbdl/rbdl.h>

namespace inria {

/**
 * Build a new RBDL model tree
 * copied from given one and
 * update the tree root.
 *
 * @param modelOld RBDL model copied from.
 * modelOld is not updated. 
 * The given model should not already 
 * contains a floating base.
 * @param newRootBodyId RBDL body Id
 * of the new root given within modelOld.
 * @param addFloatingBase If true,
 * a 6 DoFs floating base (position 
 * and spherical joint using quaternion format) 
 * is added between world origin and tree root.
 * @param indexesNewToOld Build the mapping
 * from new joint indexes in modelNew to old
 * ones in modelOld.
 * @return a new RBDL model where 
 * newRootBodyId is the new tree 
 * root after the optional floating base.
 */
RigidBodyDynamics::Model RBDLRootUpdate(
    RigidBodyDynamics::Model& modelOld, 
    size_t newRootBodyId,
    bool addFloatingBase,
    std::vector<size_t>& indexesNewToOld);

}

#endif

