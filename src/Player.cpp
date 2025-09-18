#include "Player.h"
#include "Ladder.h"
#include "CreateMaterial.h"
#include "CreatePrimitives.h"
#include "globals.h"

#include <Urho3D/Core/Timer.h>
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

using Urho3D::Time;
using Urho3D::Node;
using Urho3D::Vector3;
using Urho3D::StaticModel;
using Urho3D::RigidBody;
using Urho3D::CollisionShape;
using Urho3D::Color;
using Urho3D::E_NODECOLLISIONSTART;
namespace NodeCollisionStart = Urho3D::NodeCollisionStart;

Urho3D::SharedPtr<Urho3D::Model> Player::cylinderModel_;

Player::Player(Urho3D::Scene *scene, const Urho3D::Vector3 &pos) :
    Urho3D::Object(scene->GetContext()),
    node_(nullptr),
    walkDir_(Vector3::ZERO),
    ladder_(nullptr),
    ignoringLadder_(nullptr),
    ignoringLadderSince_(0),
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

    SubscribeToEvent(node_, E_NODECOLLISIONSTART, URHO3D_HANDLER(Player, HandleNodeCollision));
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
        if (colObj1Wrap->m_collisionObject->getUserIndex() == PhysicsUserIndex::Player)
            normal = -normal; // Player is second body
        if (normal.getY() > 0.4 && cp.getDistance() < 0.f)
        {
            if (colObj0Wrap->m_collisionObject->getUserIndex() != PhysicsUserIndex::Ladder &&
                colObj1Wrap->m_collisionObject->getUserIndex() != PhysicsUserIndex::Ladder)
            {
                onGround = true;
            }
        }
        return 0.0;
    }
    // bool onLadder{false};
    bool onGround{false};
};

void Player::Advance()
{
    RigidBody * const body = node_->GetComponent<RigidBody>();

    ContactCallback callback;
    body->GetPhysicsWorld()->GetWorld()->contactTest(body->GetBody(), callback);
    onGround_ = callback.onGround;

    if (IsOnLadder())
    {
        if (IsOnGround() && walkDir_ != Vector3::ZERO && !IsFacingLadder(walkDir_))
        {
            ignoringLadder_ = ladder_;
            ignoringLadderSince_ = Time::GetSystemTime();

            // let go of the ladder
            const Vector3 v = ladder_->GetNormalForPoint(node_->GetPosition());
            GrabLadder(nullptr);
        }
        body->Activate();
        body->SetLinearVelocity(walkDir_*PLAYER_WALK_SPEED);
    }
    else if (walkDir_ != Vector3::ZERO)
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
    if (wantJump_ && IsOnLadder())
    {
        // determine the jump-away direction
        Vector3 v = (node_->GetPosition() - ladder_->GetNode()->GetPosition());
        v.y_ = 0.0;
        v = v.Normalized()*PLAYER_JUMP_VELOCITY;

        // start ignoring this ladder for a small amount of time (so we don't re-grab it)
        ignoringLadder_ = ladder_;
        ignoringLadderSince_ = Time::GetSystemTime();

        // let go of the ladder
        GrabLadder(nullptr);

        // jump away
        body->Activate();
        body->SetLinearVelocity(v);
    }
    else if (wantJump_ && IsOnGround())
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

bool Player::IsFacingLadder(const Vector3 &faceDir) const
{
    if (!ladder_)
        return false;

    const Vector3 v = ladder_->GetNormalForPoint(node_->GetPosition());

    return faceDir.DotProduct(v) < 0.0;
}

static const unsigned IGNORING_LADDER_TIMEOUT_MS = 200;

void Player::HandleNodeCollision(Urho3D::StringHash eventType, Urho3D::VariantMap &eventData)
{
    Node * const nodeB = static_cast<Node*>(eventData[NodeCollisionStart::P_OTHERNODE].GetPtr());
    RigidBody * const bodyB = static_cast<RigidBody*>(eventData[NodeCollisionStart::P_OTHERBODY].GetPtr());
    if (nodeB && bodyB)
    {
        if (bodyB->GetBody()->getUserIndex() == PhysicsUserIndex::Ladder)
        {
            Ladder * const ladder = reinterpret_cast<Ladder*>(nodeB->GetVar("GameObjectPtr").GetVoidPtr());
            if (ladder == ignoringLadder_ && (Time::GetSystemTime() - ignoringLadderSince_ < IGNORING_LADDER_TIMEOUT_MS))
                return;
            GrabLadder(ladder);
        }
    }
}

void Player::GrabLadder(Ladder *ladder)
{
    // no change case
    if (ladder_ == ladder)
        return;

    // let go of old ladder
    if (ladder_)
        ladder_->UnconstrainNode(node_);

    // remember the ladder
    ladder_ = ladder;

    // access our physics body
    RigidBody * const body = node_->GetComponent<RigidBody>();

    // attach to the new ladder
    if (ladder)
        ladder->ConstrainNode(node_);

    // no gravity when on any ladder
    body->SetUseGravity(!ladder);
}
