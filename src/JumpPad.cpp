#include "JumpPad.h"
#include "globals.h"

#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Scene/Node.h>

#include <Urho3D/ThirdParty/Bullet/BulletDynamics/Dynamics/btRigidBody.h>

using Urho3D::Node;
using Urho3D::Vector3;
using Urho3D::RigidBody;
using Urho3D::E_NODECOLLISIONSTART;
namespace NodeCollisionStart = Urho3D::NodeCollisionStart;

JumpPad::JumpPad(Urho3D::Node *node) :
    Urho3D::Object(node->GetContext()),
    node_(node)
{
    RigidBody * const rigidBody = node_->GetComponent<RigidBody>();

    btRigidBody * const bulletBody = rigidBody->GetBody();
    bulletBody->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE); // doesn't collide with anything!
    // bulletBody->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK); // for contact added callback to be called
    bulletBody->setUserIndex(PhysicsUserIndex::JumpPad);

    SubscribeToEvent(node_, E_NODECOLLISIONSTART, URHO3D_HANDLER(JumpPad, HandleNodeCollision));
}

JumpPad::~JumpPad()
{
    node_->Remove();
    node_ = nullptr;
}

void JumpPad::HandleNodeCollision(Urho3D::StringHash eventType, Urho3D::VariantMap &eventData)
{
    Node * const nodeB = static_cast<Node*>(eventData[NodeCollisionStart::P_OTHERNODE].GetPtr());
    RigidBody * const bodyB = static_cast<RigidBody*>(eventData[NodeCollisionStart::P_OTHERBODY].GetPtr());
    if (bodyB)
    {
        Vector3 vel = bodyB->GetLinearVelocity();
        vel.y_ = 10.0;
        bodyB->SetLinearVelocity(vel);
    }
}
