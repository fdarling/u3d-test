#include "Player.h"
#include "CreateMaterial.h"
#include "CreatePrimitives.h"
#include "globals.h"

#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Scene/Scene.h>

#include <Urho3D/ThirdParty/Bullet/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
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
    node_(nullptr),
    walkDir_(Vector3::ZERO),
    onLadder_(false),
    onGround_(false),
    wantJump_(false)
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

struct ContactCallback : public btCollisionWorld::ContactResultCallback
{
    btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int, int, const btCollisionObjectWrapper* colObj1Wrap, int, int) override
    {
        btVector3 normal = cp.m_normalWorldOnB;
        if ((colObj0Wrap->m_collisionObject->getUserIndex() == PhysicsUserIndex::Ladder ||
            colObj1Wrap->m_collisionObject->getUserIndex() == PhysicsUserIndex::Ladder) &&
            normal.getY() < 0.4)
        {
            onLadder = true;
        }
        if (colObj1Wrap->m_collisionObject->getUserIndex() == PhysicsUserIndex::Player)
            normal = -normal; // Player is second body
        if (normal.getY() > 0.4 && cp.getDistance() < 0.f)
            onGround = true;
        return 0.0;
    }
    bool onLadder{false};
    bool onGround{false};
};

void Player::Advance()
{
    RigidBody * const body = node_->GetComponent<RigidBody>();

    ContactCallback callback;
    body->GetPhysicsWorld()->GetWorld()->contactTest(body->GetBody(), callback);
    onGround_ = callback.onGround;

    if (walkDir_ != Vector3::ZERO)
    {
        const Vector3 currentVelocity = body->GetLinearVelocity();
        const float mass = body->GetMass();
        const float speedInDesiredDirection = currentVelocity.DotProduct(walkDir_);
        const float walk_accel = PLAYER_WALK_ACCEL*std::max(0.0, std::min(1.0, 1.0 - speedInDesiredDirection/PLAYER_WALK_SPEED));
        const Vector3 force = mass*walk_accel*walkDir_;
        if (force != Vector3::ZERO)
        {
            body->Activate();
            body->ApplyForce(force);
            // std::cout << "force: (" << force.x_ << "," << force.y_ << "," << force.z_ << ")" << std::endl;
        }
    }
    if (wantJump_ && onGround_)
    {
        Vector3 v = body->GetLinearVelocity();
        v.y_ = PLAYER_JUMP_VELOCITY;
        body->Activate();
        body->SetLinearVelocity(v);
    }
}

void Player::SetWalkDirection(const Urho3D::Vector3 &dir)
{
    walkDir_ = dir;
}

void Player::SetJumping(bool en)
{
    wantJump_ = en;
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
