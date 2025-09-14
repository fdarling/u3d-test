#pragma once

#include <Urho3D/Container/Ptr.h>

// Urho3D forward declarations
namespace Urho3D {

class Context;
class Material;
class Color;

} // namespace Urho3D

Urho3D::SharedPtr<Urho3D::Material> CreateMaterial(Urho3D::Context *context, const Urho3D::Color &color);
