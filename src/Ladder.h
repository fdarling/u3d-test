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
    Ladder(Urho3D::Scene *scene, const Urho3D::Vector3 &pos, const Urho3D::Vector3 &size);
    ~Ladder();
    
    void ConstrainNode(Urho3D::Node *otherNode);
    void UnconstrainNode(Urho3D::Node *otherNode);
protected:
    typedef std::unordered_map<Urho3D::Node*, btGeneric6DofConstraint*> ConstraintMap;
    Urho3D::Node *node_;
    Urho3D::RigidBody *body_;
    ConstraintMap constrainedNodes_;
};
