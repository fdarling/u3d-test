#include "Elevator.h"
#include "KinematicRigidBody.h"
#include "globals.h"

#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/PhysicsUtils.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Scene/Node.h>

#include <Urho3D/ThirdParty/Bullet/BulletDynamics/Dynamics/btRigidBody.h>
#include <Urho3D/ThirdParty/Bullet/LinearMath/btTransformUtil.h>

using Urho3D::Node;
using Urho3D::Vector3;
using Urho3D::Quaternion;
using Urho3D::ToVector3;
using Urho3D::ToQuaternion;
using Urho3D::PhysicsWorld;
using Urho3D::RigidBody;
using Urho3D::E_POSTUPDATE;
using Urho3D::E_PHYSICSPRESTEP;
using Urho3D::E_NODECOLLISIONSTART;
namespace PostUpdate = Urho3D::PostUpdate;
namespace PhysicsPreStep = Urho3D::PhysicsPreStep;
namespace BeginFrame = Urho3D::BeginFrame;
namespace NodeCollisionStart = Urho3D::NodeCollisionStart;

static const float ELEVATOR_SPEED = 5.0f;
// static const Vector3 ELEVATOR_VEL = Vector3::UP*ELEVATOR_SPEED; // with U3D (not rbfx), this evaluates to Urho3D::Vector3::ZERO somehow... maybe a static global variable initialization order issue?
static const Vector3 ELEVATOR_VEL = Vector3(0, 1, 0)*ELEVATOR_SPEED;

Elevator::Elevator(Urho3D::Node *node) :
    Urho3D::Object(node->GetContext()),
    node_(node),
    _elevating(false),
    _accumulator(0.0)
{
    RigidBody * const rigidBody = node_->GetComponent<RigidBody>();
    rigidBody->getWorldTransform(_oldTransform);

    btRigidBody * const bulletBody = rigidBody->GetBody();
    bulletBody->setCollisionFlags(bulletBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT); // can be programmatically moved
    bulletBody->setActivationState(DISABLE_DEACTIVATION); // TODO is this necessary?
    bulletBody->setUserIndex(PhysicsUserIndex::Elevator);

    PhysicsWorld * const world = node_->GetScene()->GetComponent<PhysicsWorld>();
    SubscribeToEvent(E_POSTUPDATE, URHO3D_HANDLER(Elevator, HandlePostUpdate));
    // SubscribeToEvent(world, E_PHYSICSPREUPDATE, URHO3D_HANDLER(Elevator, HandlePhysicsPreUpdate));
    // SubscribeToEvent(world, E_PHYSICSPOSTUPDATE, URHO3D_HANDLER(Elevator, HandlePhysicsPostUpdate));
    SubscribeToEvent(world, E_PHYSICSPRESTEP, URHO3D_HANDLER(Elevator, HandlePhysicsPreStep));
    // SubscribeToEvent(world, E_PHYSICSPOSTSTEP, URHO3D_HANDLER(Elevator, HandlePhysicsPostStep));
    SubscribeToEvent(node_, E_NODECOLLISIONSTART, URHO3D_HANDLER(Elevator, HandleNodeCollisionStart));
}

Elevator::~Elevator()
{
    node_->Remove();
    node_ = nullptr;
}

void Elevator::HandlePostUpdate(Urho3D::StringHash eventType, Urho3D::VariantMap &eventData)
{
    // std::cout << "Elevator::HandlePostUpdate" << std::endl;
    if (_elevating)
    {
        _accumulator += eventData[PostUpdate::P_TIMESTEP].GetFloat();

        // interpolate from physics pose to graphics pose
        KinematicRigidBody * const ourBody = static_cast<KinematicRigidBody*>(node_->GetComponent<RigidBody>());
        btRigidBody * const body = ourBody->GetBody();
        btTransform interpolatedTrans;
        btTransformUtil::integrateTransform(_oldTransform,
                                            body->getInterpolationLinearVelocity(), body->getInterpolationAngularVelocity(),
                                            _accumulator,
                                            interpolatedTrans);
        const Quaternion interpolatedRotation = ToQuaternion(interpolatedTrans.getRotation());
        const Vector3 interpolatedPosition = ToVector3(interpolatedTrans.getOrigin()) - interpolatedRotation * ourBody->GetCenterOfMass();

        // update the pose that is *only used by graphics*!
        ourBody->GetPhysicsWorld()->SetApplyingTransforms(true);
        node_->SetWorldPosition(interpolatedPosition);
        node_->SetWorldRotation(interpolatedRotation);
        // lastPosition_ = node_->GetWorldPosition();
        // lastRotation_ = node_->GetWorldRotation();
        ourBody->GetPhysicsWorld()->SetApplyingTransforms(false);
    }
}

void Elevator::HandlePhysicsPreStep(Urho3D::StringHash eventType, Urho3D::VariantMap &eventData)
{
    // std::cout << "HandlePhysicsPreStep" << std::endl;
    if (_elevating)
    {
        const float timeStep = eventData[PhysicsPreStep::P_TIMESTEP].GetFloat();
        _accumulator -= timeStep;

        KinematicRigidBody * const ourBody = static_cast<KinematicRigidBody*>(node_->GetComponent<RigidBody>());
        ourBody->getWorldTransform(_oldTransform); // we save this for interpolation purposes

        ////// const Vector3 pos = ourBody->GetPosition();
        // const Quaternion rot = ToQuaternion(_oldTransform.getRotation());
        // const Vector3 pos = ToVector3(_oldTransform.getOrigin()) - rot * ourBody->GetCenterOfMass();
        // const Vector3 newPos = pos + ELEVATOR_VEL*timeStep;
        // const Vector3 vel = ourBody->GetLinearVelocity();

        btTransform newPhysicsTrans;
        newPhysicsTrans.setIdentity();
        //// newPhysicsTrans.setOrigin(ToBtVector3(newPos + rot * ourBody->GetCenterOfMass()));
        //// newPhysicsTrans.setRotation(ToBtQuaternion(rot));
        newPhysicsTrans.setOrigin(_oldTransform.getOrigin() + ToBtVector3(ELEVATOR_VEL)*timeStep);
        newPhysicsTrans.setRotation(_oldTransform.getRotation());
        
        // update the pose that is *only used by graphics*!
        ourBody->setOverrideTransform(newPhysicsTrans);
        // update the rest of the physics state
        ourBody->Activate();
        ourBody->SetLinearVelocity(ELEVATOR_VEL);
        // std::cout << "Elevator::HandlePhysicsPreStep: pos = " << pos.ToString().c_str() << "; vel = " << vel.ToString().c_str() << std::endl;
        // std::cout << "Elevator::HandlePhysicsPreStep: pos = " << pos.ToString().c_str() << "; newPos = " << newPos.ToString().c_str() << std::endl;
    }
}

void Elevator::HandleNodeCollisionStart(Urho3D::StringHash eventType, Urho3D::VariantMap &eventData)
{
    // Node * const otherNode = static_cast<Node*>(eventData[NodeCollisionStart::P_OTHERNODE].GetPtr());
    RigidBody * const otherBody = static_cast<RigidBody*>(eventData[NodeCollisionStart::P_OTHERBODY].GetPtr());
    if (otherBody && otherBody->GetBody()->getUserIndex() == PhysicsUserIndex::Player)
    {
        // std::cout << "TOUCHED ELEVATOR" << std::endl;
        _elevating = true;
        _accumulator = 0.0;
    }
}
