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

static const float DEST_COOLDOWN = 2.0;
static const float MAX_ELEVATOR_TRAVEL = 40.0f;
static const float ELEVATOR_SPEED = 5.0f;

Elevator::Elevator(Urho3D::Node *node) :
    Urho3D::Object(node->GetContext()),
    node_(node),
    _state(State::Idle),
    _accumulator(0.0),
    _cooldown(0.0)
{
    RigidBody * const rigidBody = node_->GetComponent<RigidBody>();
    rigidBody->getWorldTransform(_oldTransform);

    btRigidBody * const bulletBody = rigidBody->GetBody();
    bulletBody->setCollisionFlags(bulletBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT); // can be programmatically moved
    bulletBody->setActivationState(DISABLE_DEACTIVATION); // TODO is this necessary?
    bulletBody->setUserIndex(PhysicsUserIndex::Elevator);

    PhysicsWorld * const world = node_->GetScene()->GetComponent<PhysicsWorld>();
    // TODO call SubscribeToEvent() and UnsubscribeFromEvent() as needed so they aren't always firing
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
    if (_state != State::Idle)
    {
        _accumulator += eventData[PostUpdate::P_TIMESTEP].GetFloat();

        // interpolate graphics version of transform using old physics transform and velocity/rotation
        KinematicRigidBody * const ourBody = static_cast<KinematicRigidBody*>(node_->GetComponent<RigidBody>());
        btRigidBody * const body = ourBody->GetBody();
        btTransform interpolatedTrans;
        btTransformUtil::integrateTransform(_oldTransform,
                                            body->getInterpolationLinearVelocity(), body->getInterpolationAngularVelocity(),
                                            _accumulator,
                                            interpolatedTrans);
        const Quaternion interpolatedRotation = ToQuaternion(interpolatedTrans.getRotation());
        const Vector3 interpolatedPosition = ToVector3(interpolatedTrans.getOrigin()) - interpolatedRotation * ourBody->GetCenterOfMass();

        // update the transform used by the graphics system
        ourBody->GetPhysicsWorld()->SetApplyingTransforms(true);
        node_->SetWorldPosition(interpolatedPosition);
        node_->SetWorldRotation(interpolatedRotation);
        ourBody->GetPhysicsWorld()->SetApplyingTransforms(false);
    }
}

void Elevator::HandlePhysicsPreStep(Urho3D::StringHash eventType, Urho3D::VariantMap &eventData)
{
    // std::cout << "HandlePhysicsPreStep" << std::endl;
    if (_state != State::Idle)
    {
        const float timeStep = eventData[PhysicsPreStep::P_TIMESTEP].GetFloat();
        _accumulator -= timeStep;

        KinematicRigidBody * const ourBody = static_cast<KinematicRigidBody*>(node_->GetComponent<RigidBody>());

        // save current (will become "old") physics transform for graphics interpolation purposes
        ourBody->getWorldTransform(_oldTransform); // we save this for interpolation purposes

        if (_state == State::Departing || _state == State::Returning)
        {
            // calculate the direction of motion and velocity
            const Vector3 startToEndVec = Vector3(_endTrans.getOrigin() - _startTrans.getOrigin()).Normalized();
            Vector3 newVel = ((_state == State::Departing) ? startToEndVec : -startToEndVec)*ELEVATOR_SPEED;

            // calculate new physics transform
            btTransform newPhysicsTrans;
            newPhysicsTrans.setIdentity();
            newPhysicsTrans.setOrigin(_oldTransform.getOrigin() + ToBtVector3(newVel)*timeStep);
            newPhysicsTrans.setRotation(_oldTransform.getRotation());

            // cap the velocity / target position to not overshoot
            const btTransform &targetTrans = (_state == State::Departing) ? _endTrans : _startTrans;
            const btScalar targetRelativeY = (newPhysicsTrans.getOrigin().y() - targetTrans.getOrigin().y());
            const bool would_overshoot = (newVel.y_ > 0.0) ? (targetRelativeY >= 0.0) : (targetRelativeY <= 0.0);
            if (would_overshoot)
            {
                newPhysicsTrans = targetTrans;
                newVel = Vector3(targetTrans.getOrigin() - _oldTransform.getOrigin())/timeStep;
                if (_state == State::Departing)
                    _state = State::DestCooldown;
                else
                    _state = State::OriginCooldown;
                _cooldown = DEST_COOLDOWN;
            }

            // update the transform that is *used by physics*!
            ourBody->setOverrideTransform(newPhysicsTrans);
            // update the rest of the physics state
            ourBody->Activate();
            ourBody->SetLinearVelocity(newVel);

            // EVIL HACK to effectively get per-substep kinematic body information into Bullet, rather than per "full step"
            btRigidBody * const body = ourBody->GetBody();
            body->getInterpolationWorldTransform() = _oldTransform;
            body->saveKinematicState(timeStep);
        }
        else if (_state == State::DestCooldown || _state == State::OriginCooldown)
        {
            _cooldown -= timeStep;
            if (_cooldown <= 0.0)
            {
                if (_state == State::DestCooldown)
                    _state = State::Returning;
                else
                    _state = State::Idle;
            }
        }
        // std::cout << "Elevator::HandlePhysicsPreStep: pos = " << pos.ToString().c_str() << "; vel = " << vel.ToString().c_str() << std::endl;
        // std::cout << "Elevator::HandlePhysicsPreStep: pos = " << pos.ToString().c_str() << "; newPos = " << newPos.ToString().c_str() << std::endl;
    }
}

void Elevator::HandleNodeCollisionStart(Urho3D::StringHash eventType, Urho3D::VariantMap &eventData)
{
    if (_state != State::Idle)
        return;
    // Node * const otherNode = static_cast<Node*>(eventData[NodeCollisionStart::P_OTHERNODE].GetPtr());
    RigidBody * const otherBody = static_cast<RigidBody*>(eventData[NodeCollisionStart::P_OTHERBODY].GetPtr());
    if (otherBody && otherBody->GetBody()->getUserIndex() == PhysicsUserIndex::Player)
    {
        // std::cout << "TOUCHED ELEVATOR" << std::endl;
        _state = State::Departing;
        _accumulator = 0.0;
        KinematicRigidBody * const ourBody = static_cast<KinematicRigidBody*>(node_->GetComponent<RigidBody>());
        ourBody->getWorldTransform(_startTrans);
        _endTrans = _startTrans;
        _endTrans.setOrigin(_startTrans.getOrigin() + ToBtVector3(Vector3::UP*MAX_ELEVATOR_TRAVEL));
    }
}
