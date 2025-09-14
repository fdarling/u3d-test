#pragma once

#include <Urho3D/Container/Ptr.h>

// Urho3D forward declarations
namespace Urho3D {

class Model;
class Node;
class Scene;
class Vector3;
class Color;

} // namespace Urho3D

class Ball
{
public:
    Ball(Urho3D::Scene *scene, const Urho3D::Vector3 &pos, const Urho3D::Vector3 &vel, const Urho3D::Color &color);
    ~Ball();
    Urho3D::Node * GetNode() {return node_;}
    const Urho3D::Node * GetNode() const {return node_;}
protected:
    Urho3D::Node *node_;
    static Urho3D::SharedPtr<Urho3D::Model> sphereModel_;
};
