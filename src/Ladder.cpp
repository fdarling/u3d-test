#include "Ladder.h"
#include "globals.h"

#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Physics/PhysicsUtils.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/CollisionShape.h>

#include <Urho3D/ThirdParty/Bullet/BulletCollision/CollisionShapes/btCollisionShape.h>
#include <Urho3D/ThirdParty/Bullet/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <Urho3D/ThirdParty/Bullet/BulletDynamics/Dynamics/btRigidBody.h>
#include <Urho3D/ThirdParty/Bullet/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>

#include <array> // for std::array<>
#include <algorithm> // for std::max_element()
#include <limits> // for std::numeric_limits<>

using Urho3D::Vector3;
using Urho3D::BoundingBox;
using Urho3D::RigidBody;
using Urho3D::CollisionShape;
using Urho3D::ToBtVector3;

Ladder::Ladder(Urho3D::Node *node) :
    node_(node),
    body_(node_->GetComponent<RigidBody>())
{
    btRigidBody * const body = body_->GetBody();
    body->setUserIndex(PhysicsUserIndex::Ladder);
    node_->SetVar("GameObjectPtr", this);
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

Vector3 Ladder::GetNormalForPoint(const Urho3D::Vector3 &pt) const
{
    // calculate "cylindrical" normal
    Vector3 v = pt - node_->GetPosition();
    v.y_ = 0.0; // remove the vertical component
    if (v == Vector3::ZERO)
        return Vector3::ZERO;
    v.Normalize();

    // find which cardinal direction best matches the vector
    // TODO: take the node's rotation into consideration...
    static const std::array<Vector3, 4> CARDINAL_DIRECTIONS{
        Vector3::LEFT,
        Vector3::RIGHT,
        Vector3::FORWARD,
        Vector3::BACK
    };
    std::array<float, 4> dotProducts;
    std::transform(CARDINAL_DIRECTIONS.begin(), CARDINAL_DIRECTIONS.end(), dotProducts.begin(), [&] (const Vector3 &dir) {
        return v.DotProduct(dir);
    });
    std::array<float, 4>::const_iterator bestIt = std::max_element(dotProducts.cbegin(), dotProducts.cend());
    std::size_t bestIndex = std::distance(dotProducts.cbegin(), bestIt);

    // use that cardinal direction
    return CARDINAL_DIRECTIONS[bestIndex];
}

static Urho3D::BoundingBox GetLocalAABB(Urho3D::CollisionShape *collisionShape)
{
    btCollisionShape * const shape = collisionShape->GetCollisionShape();
    btTransform localTransform;
    localTransform.setIdentity();
    btVector3 aabbMin;
    btVector3 aabbMax;
    shape->getAabb(localTransform, aabbMin, aabbMax);
    return BoundingBox(Vector3(aabbMin), Vector3(aabbMax));
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
    const Vector3 v = node_->GetScale()*node_->GetComponent<CollisionShape>()->GetSize()/2.0f;
    const BoundingBox shapeBB = GetLocalAABB(node_->GetComponent<CollisionShape>());
    static const btScalar TOLERANCE = 0.05;
    const btVector3 expansion3D = btVector3(PLAYER_RADIUS + TOLERANCE, PLAYER_HEIGHT/2.0 + TOLERANCE, PLAYER_RADIUS + TOLERANCE);
    constraint->setLinearLowerLimit(ToBtVector3(shapeBB.min_) - expansion3D);
    constraint->setLinearUpperLimit(ToBtVector3(shapeBB.max_) + expansion3D);

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
