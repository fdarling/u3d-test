#pragma once

#include <Urho3D/Core/Object.h>
#include <Urho3D/Core/Variant.h>

// forward declarations
namespace Urho3D {

class Scene;
class Model;
class Node;
class Vector3;
class StringHash;

} // namespace Urho3D

// forward declaration
class Ladder;

class Player : public Urho3D::Object
{
    URHO3D_OBJECT(Player, Urho3D::Object);
public:
    Player(Urho3D::Scene *scene, const Urho3D::Vector3 &pos);
    ~Player();

    void Advance();
    void SetWalkAndFlyDirections(const Urho3D::Vector3 &walkDir, const Urho3D::Vector3 &flyDir);
    void SetJumping(bool en);
    bool IsOnLadder() const {return ladder_;}
    bool IsOnGround() const {return onGround_;}
    bool IsFacingLadder(const Urho3D::Vector3 &faceDir) const;
    bool IsAboveLadderVertically() const;
    bool IsAboveLadderHorizontally() const;
    Urho3D::Vector3 GetLadderNormal() const;
    Urho3D::Node * GetNode() {return node_;}
    const Urho3D::Node * GetNode() const {return node_;}
protected:
    void HandleNodeCollision(Urho3D::StringHash eventType, Urho3D::VariantMap &eventData);
    void GrabLadder(Ladder *ladder);

    static Urho3D::SharedPtr<Urho3D::Model> cylinderModel_;

    Urho3D::Node *node_;
    Urho3D::Vector3 walkDir_;
    Urho3D::Vector3 flyDir_;
    Ladder *ladder_;
    bool onGround_;
    bool wantJump_;
};
