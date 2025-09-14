#include "Ladder.h"
#include "globals.h"

#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/CollisionShape.h>

#include <Urho3D/ThirdParty/Bullet/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <Urho3D/ThirdParty/Bullet/BulletDynamics/Dynamics/btRigidBody.h>
#include <Urho3D/ThirdParty/Bullet/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>

using Urho3D::Vector3;
using Urho3D::RigidBody;
using Urho3D::CollisionShape;

Ladder::Ladder(Urho3D::Scene *scene, const Urho3D::Vector3 &pos, const Urho3D::Vector3 &size) :
    node_(nullptr),
    body_(nullptr)
{
    node_ = scene->CreateChild("Ladder");
    node_->SetPosition(pos);

    body_ = node_->CreateComponent<RigidBody>();
    CollisionShape * const ladderShape = node_->CreateComponent<CollisionShape>();
    ladderShape->SetBox(size);

    btRigidBody * const body = body_->GetBody();
    body->setUserIndex(PhysicsUserIndex::Ladder);
}

Ladder::~Ladder()
{
    for (ConstraintMap::iterator it = constrainedNodes_.begin(); it != constrainedNodes_.end(); ++it)
        delete it->second;
    constrainedNodes_.clear();
    node_->Remove();
    node_ = nullptr;
    body_ = nullptr;
}

void Ladder::ConstrainNode(Urho3D::Node *otherNode)
{
    // make sure we didn't already constrain it
    if (constrainedNodes_.find(otherNode) != constrainedNodes_.end())
        return;

    // make sure the node has a physics body
    RigidBody * const rigidBodyB = otherNode->GetComponent<RigidBody>();
    if (!rigidBodyB)
        return;

    // access the lower level Bullet bodies
    btRigidBody * const bodyA = body_->GetBody();
    btRigidBody * const bodyB = rigidBodyB->GetBody();

    // reference frames (currently identity)
    btTransform frameInA;
    frameInA.setIdentity();
    btTransform frameInB;
    frameInB.setIdentity();

    // create the constraint for limit the distance from the ladder
    btGeneric6DofConstraint * const constraint = new btGeneric6DofConstraint(
        *bodyA,
        *bodyB,
        frameInA,
        frameInB,
        true // useLinearReferenceFrameA
    );

    // partial XYZ volume around "ladder"
    // TODO use node_->GetComponent<CollisionShape>()->Size()
    constraint->setLinearLowerLimit(btVector3(-1.25, -8, -1.25));
    constraint->setLinearUpperLimit(btVector3( 1.25,  8,  1.25));

    // only allow rotation about the Z axis
    // constraint->setAngularLowerLimit(btVector3(0, -SIMD_INFINITY, 0));
    // constraint->setAngularUpperLimit(btVector3(0, SIMD_INFINITY, 0));

    // allow all rotation
    constraint->setAngularLowerLimit(btVector3(-SIMD_INFINITY, -SIMD_INFINITY, -SIMD_INFINITY));
    constraint->setAngularUpperLimit(btVector3( SIMD_INFINITY,  SIMD_INFINITY,  SIMD_INFINITY));

    // actually add the constraint
    btDiscreteDynamicsWorld * const world = body_->GetPhysicsWorld()->GetWorld();
    world->addConstraint(constraint, false); // the bool is for whether the constrained bodies are exempt from colliding with each other

    // remember that we constrained it
    constrainedNodes_.insert({otherNode, constraint});
}

void Ladder::UnconstrainNode(Urho3D::Node *otherNode)
{
    // make sure we actually have it
    ConstraintMap::iterator it = constrainedNodes_.find(otherNode);
    if (it == constrainedNodes_.end())
        return;

    // remove the constraint from the system
    btDiscreteDynamicsWorld * const world = body_->GetPhysicsWorld()->GetWorld();
    world->removeConstraint(it->second);

    // delete the constraint
    delete it->second;

    // forget the constraint
    constrainedNodes_.erase(it);
}
