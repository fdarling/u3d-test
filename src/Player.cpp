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
using Urho3D::BoundingBox;
using Urho3D::Color;
using Urho3D::Clamp;
using Urho3D::Quaternion;
using Urho3D::OUTSIDE;
using Urho3D::E_NODECOLLISIONSTART;
namespace NodeCollisionStart = Urho3D::NodeCollisionStart;

Urho3D::SharedPtr<Urho3D::Model> Player::cylinderModel_;

Player::Player(Urho3D::Scene *scene, const Urho3D::Vector3 &pos) :
    Urho3D::Object(scene->GetContext()),
    node_(nullptr),
    walkDir_(Vector3::ZERO),
    ladder_(nullptr),
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

    SubscribeToEvent(node_, E_NODECOLLISIONSTART, URHO3D_HANDLER(Player, HandleNodeCollisionStart));
}

Player::~Player()
{
    node_->Remove();
    node_ = nullptr;
}

static Vector3 adjustWalkDir(Player *player, const Vector3 &walkDir)
{
    // leave pitch unmodified if we aren't even on a ladder
    if (!player->IsOnLadder())
        return walkDir;

    // NOTE: on the ground and trying to leave the ladder case already handled outside this function!

    // check to see if we are at the top of the ladder (maximum altitude)
    const bool aboveLadderVertically = player->IsAboveLadderVertically();
    if (aboveLadderVertically)
    {
        // are we on top of the ladder (in the sense of it being a platform)?
        const bool aboveLadderHorizontally = player->IsAboveLadderHorizontally();
        if (aboveLadderHorizontally)
            return walkDir;

        // are we walking onto the top of the ladder?
        const bool walkingTowardsLadder = player->IsFacingLadder(walkDir);
        if (walkingTowardsLadder)
            return walkDir;
    }

    // we are not at the top of the ladder, or we are at the top but
    // trying to climb down not up
    const Vector3 ladderNormal = player->GetLadderNormal();
    const Vector3 rotAxis = ladderNormal.CrossProduct(Vector3::UP);
    const float normalDot = ladderNormal.DotProduct(walkDir);
    const Vector3 normalComponent = (ladderNormal*normalDot);
    if (normalComponent == Vector3::ZERO)
        return walkDir;
    const Vector3 verticalComponent = Vector3::UP*(walkDir.DotProduct(Vector3::UP));
    const float normalPitch = (verticalComponent + normalComponent).Angle(-ladderNormal);
    const float targetPitch = Clamp(2.0f*(normalPitch - 45.0f), -89.9f, 89.9f);
    const Vector3 newWalkDir = Quaternion(targetPitch - normalPitch, rotAxis).RotationMatrix()*walkDir;

    return newWalkDir;
};

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

    // test if we are on the ground
    ContactCallback callback;
    body->GetPhysicsWorld()->GetWorld()->contactTest(body->GetBody(), callback);
    onGround_ = callback.onGround;

    // handle special ladder behavior
    if (IsOnLadder())
    {
        // if we are on the ground and generally walking away from the ladder, depart!
        if (IsOnGround() && walkDir_ != Vector3::ZERO && !IsFacingLadder(walkDir_))
        {
            // let go of the ladder
            GrabLadder(nullptr);
        }

        // otherwise, commute horizontal "into" / "out-of" the ladder movement into vertical motion
        const Vector3 adjustedDir = adjustWalkDir(this, flyDir_);

        // when on the ladder, we move at a constant speed (rather than accelerate)
        body->Activate();
        body->SetLinearVelocity(adjustedDir*PLAYER_WALK_SPEED);
    }
    else if (walkDir_ != Vector3::ZERO)
    {
        const Vector3 currentVelocity = body->GetLinearVelocity();
        const float mass = body->GetMass();
        const float speedInDesiredDirection = currentVelocity.DotProduct(walkDir_);
        const float walk_accel = PLAYER_WALK_ACCEL*Clamp(1.0 - speedInDesiredDirection/PLAYER_WALK_SPEED, 0.0, 1.0);
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
        const Vector3 v = ladder_->GetNormalForPoint(node_->GetPosition())*PLAYER_JUMP_VELOCITY;

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

void Player::SetWalkAndFlyDirections(const Urho3D::Vector3 &walkDir, const Urho3D::Vector3 &flyDir)
{
    walkDir_ = walkDir;
    flyDir_ = flyDir;
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

    return faceDir.DotProduct(v) < 0.0f;
}

static const float OVERLAP_TOLERANCE = 0.05;
static const Vector3 HORIZONTAL_OVERLAP_TOLERANCE(OVERLAP_TOLERANCE, 0.0f, OVERLAP_TOLERANCE);

bool Player::IsAboveLadderVertically() const
{
    if (!ladder_)
        return false;
    CollisionShape * const playerShape = node_->GetComponent<CollisionShape>();
    CollisionShape * const ladderShape = ladder_->GetNode()->GetComponent<CollisionShape>();
    const BoundingBox playerBB = playerShape->GetWorldBoundingBox();
    const BoundingBox ladderBB = ladderShape->GetWorldBoundingBox();
    return playerBB.min_.y_ + OVERLAP_TOLERANCE >= ladderBB.max_.y_;
}

bool Player::IsAboveLadderHorizontally() const
{
    if (!ladder_)
        return false;
    CollisionShape * const playerShape = node_->GetComponent<CollisionShape>();
    CollisionShape * const ladderShape = ladder_->GetNode()->GetComponent<CollisionShape>();
    const BoundingBox playerBB = playerShape->GetWorldBoundingBox();
          BoundingBox ladderBB = ladderShape->GetWorldBoundingBox();

    // shrink ladder bounding box horizontally
    ladderBB.min_ += HORIZONTAL_OVERLAP_TOLERANCE;
    ladderBB.max_ -= HORIZONTAL_OVERLAP_TOLERANCE;

    // grow ladder bounding box vertically by height of player
    ladderBB.max_.y_ += playerBB.Size().y_;

    return ladderBB.IsInside(playerBB) != OUTSIDE;
}

Urho3D::Vector3 Player::GetLadderNormal() const
{
    if (!ladder_)
        return Vector3::ZERO;
    return ladder_->GetNormalForPoint(node_->GetPosition());
}

void Player::HandleNodeCollisionStart(Urho3D::StringHash eventType, Urho3D::VariantMap &eventData)
{
    Node * const nodeB = static_cast<Node*>(eventData[NodeCollisionStart::P_OTHERNODE].GetPtr());
    RigidBody * const bodyB = static_cast<RigidBody*>(eventData[NodeCollisionStart::P_OTHERBODY].GetPtr());
    if (nodeB && bodyB)
    {
        if (bodyB->GetBody()->getUserIndex() == PhysicsUserIndex::Ladder)
        {
            Ladder * const ladder = reinterpret_cast<Ladder*>(nodeB->GetVar("GameObjectPtr").GetVoidPtr());
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
