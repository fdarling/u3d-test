#include "Player.h"
#include "CreateMaterial.h"
#include "CreatePrimitives.h"
#include "globals.h"

#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Scene/Scene.h>

#include <Urho3D/ThirdParty/Bullet/BulletDynamics/Dynamics/btRigidBody.h>

using Urho3D::Node;
using Urho3D::Vector3;
using Urho3D::StaticModel;
using Urho3D::RigidBody;
using Urho3D::CollisionShape;
using Urho3D::Color;
using Urho3D::E_NODECOLLISION;
namespace NodeCollisionStart = Urho3D::NodeCollisionStart;

Urho3D::SharedPtr<Urho3D::Model> Player::cylinderModel_;

Player::Player(Urho3D::Scene *scene, const Urho3D::Vector3 &pos) :
    Urho3D::Object(scene->GetContext()),
    node_(nullptr)
{
    node_ = scene->CreateChild("Player");
    node_->SetPosition(pos);

    // possibly create and cache the model
    if (!cylinderModel_)
        cylinderModel_ = CreateCapsuleModel(scene->GetContext(), PLAYER_RADIUS, PLAYER_HEIGHT-2.0*PLAYER_RADIUS); // TODO support multiple contexts!

    // use the model
    StaticModel * const sm = node_->CreateComponent<StaticModel>();
    sm->SetModel(cylinderModel_);
    sm->SetMaterial(CreateMaterial(scene->GetContext(), Color(0.8, 0.8, 0.8)));
    sm->SetCastShadows(true);

    // create physics body
    RigidBody * const body = node_->CreateComponent<RigidBody>();
    body->SetMass(PLAYER_MASS);
    body->SetFriction(0.5f);
    body->SetLinearDamping(0.2f);
    body->SetAngularDamping(0.2f);
    body->SetAngularFactor(Vector3(0, 0, 0)); // prevent tipping over

    // create physics shape
    CollisionShape * const shape = node_->CreateComponent<CollisionShape>();
    shape->SetCapsule(PLAYER_RADIUS*2.0, PLAYER_HEIGHT);

    btRigidBody * const bulletBody = body->GetBody();
    bulletBody->setUserIndex(PhysicsUserIndex::Player);

    node_->SubscribeToEvent(node_, E_NODECOLLISION, URHO3D_HANDLER(Player, HandleNodeCollision));
}

Player::~Player()
{
    node_->Remove();
    node_ = nullptr;
}

void Player::HandleNodeCollision(Urho3D::StringHash eventType, Urho3D::VariantMap &eventData)
{
    Node * const nodeB = static_cast<Node*>(eventData[NodeCollisionStart::P_OTHERNODE].GetPtr());
    RigidBody * const bodyB = static_cast<RigidBody*>(eventData[NodeCollisionStart::P_OTHERBODY].GetPtr());
    if (bodyB)
    {
        if (bodyB->GetBody()->getUserIndex() == PhysicsUserIndex::Ladder)
        {
            // std::cout << "\t\ttouching a Ladder!" << std::endl;
        }
    }
}
