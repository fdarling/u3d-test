#include "CreateMaterial.h"

#ifdef USING_RBFX
#include <Urho3D/RenderPipeline/ShaderConsts.h>
#endif // USING_RBFX
#include <Urho3D/Core/Context.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Technique.h>
#include <Urho3D/Resource/ResourceCache.h>

using Urho3D::SharedPtr;
using Urho3D::ResourceCache;
using Urho3D::Material;
using Urho3D::Technique;
#ifdef USING_RBFX
using Urho3D::ShaderConsts::Material_MatDiffColor;
#endif
using Urho3D::CULL_CW;

Urho3D::SharedPtr<Urho3D::Material> CreateMaterial(Urho3D::Context *context, const Urho3D::Color &color)
{
    ResourceCache * const cache = context->GetSubsystem<ResourceCache>();
    SharedPtr<Material> mat(new Material(context));
    mat->SetTechnique(0, cache->GetResource<Technique>("Techniques/NoTextureAO.xml"));
#ifdef USING_RBFX
    mat->SetShaderParameter(Material_MatDiffColor, color);
#else // USING_RBFX
    mat->SetShaderParameter("MatDiffColor", color);
#endif // USING_RBFX
    mat->SetShadowCullMode(CULL_CW);
    return mat;
}
