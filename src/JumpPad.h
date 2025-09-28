#pragma once

#include <Urho3D/Core/Object.h>
#include <Urho3D/Core/Variant.h>

// forward declarations
namespace Urho3D {

class Node;
class Vector3;
class StringHash;

} // namespace Urho3D

class JumpPad : public Urho3D::Object
{
    URHO3D_OBJECT(JumpPad, Urho3D::Object);
public:
    JumpPad(Urho3D::Node *node);
    ~JumpPad();
protected:
    void HandleNodeCollision(Urho3D::StringHash eventType, Urho3D::VariantMap &eventData);

    Urho3D::Node *node_;
};
