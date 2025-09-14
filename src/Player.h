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

class Player : public Urho3D::Object
{
    URHO3D_OBJECT(Player, Urho3D::Object);
public:
    Player(Urho3D::Scene *scene, const Urho3D::Vector3 &pos);
    ~Player();
protected:
    void HandleNodeCollision(Urho3D::StringHash eventType, Urho3D::VariantMap &eventData);

    Urho3D::Node *node_;
    static Urho3D::SharedPtr<Urho3D::Model> cylinderModel_;
};
