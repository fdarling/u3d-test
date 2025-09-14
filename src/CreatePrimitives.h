#pragma once

#include <Urho3D/Container/Ptr.h>

// Urho3D forward declarations
namespace Urho3D {

class Context;
class Model;

} // namespace Urho3D

Urho3D::SharedPtr<Urho3D::Model> CreateSphereModel(Urho3D::Context *context, float radius = 0.25f, int stacks = 16, int slices = 16);
Urho3D::SharedPtr<Urho3D::Model> CreateCapsuleModel(Urho3D::Context *context, float radius, float height, int rings = 16, int segments = 16);
