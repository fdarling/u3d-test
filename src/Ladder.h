#pragma once

#include <unordered_map>

// Urho3D forward declarations
namespace Urho3D {

class Scene;
class Node;
class RigidBody;
class Vector3;

} // namespace Urho3D

// Bullet forward declarations
class btGeneric6DofConstraint;

class Ladder
{
public:
    Ladder(Urho3D::Node *node);
    ~Ladder();

    Urho3D::Vector3 GetNormalForPoint(const Urho3D::Vector3 &pt) const;

    void ConstrainNode(Urho3D::Node *otherNode);
    void UnconstrainNode(Urho3D::Node *otherNode);

    Urho3D::Node * GetNode() {return node_;}
    const Urho3D::Node * GetNode() const {return node_;}
protected:
    typedef std::unordered_map<Urho3D::Node*, btGeneric6DofConstraint*> ConstraintMap;
    Urho3D::Node *node_;
    Urho3D::RigidBody *body_;
    ConstraintMap constrainedNodes_;
};
