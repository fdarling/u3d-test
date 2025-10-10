#pragma once

#include <Urho3D/Core/Object.h>
#include <Urho3D/Core/Variant.h>

#include <Urho3D/ThirdParty/Bullet/LinearMath/btTransform.h>

// forward declarations
namespace Urho3D {

class Node;
class Vector3;
class StringHash;

} // namespace Urho3D

class Elevator : public Urho3D::Object
{
    URHO3D_OBJECT(Elevator, Urho3D::Object);
public:
    Elevator(Urho3D::Node *node);
    ~Elevator();
protected:
    void HandlePostUpdate(Urho3D::StringHash eventType, Urho3D::VariantMap &eventData);
    void HandlePhysicsPreStep(Urho3D::StringHash eventType, Urho3D::VariantMap &eventData);
    void HandleNodeCollisionStart(Urho3D::StringHash eventType, Urho3D::VariantMap &eventData);

    Urho3D::Node *node_;
    bool _elevating;
    float _accumulator;
    btTransform _oldTransform;
};
