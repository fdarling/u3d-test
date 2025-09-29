#include <Urho3D/Math/MathDefs.h>
#include <Urho3D/Core/Context.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/VertexBuffer.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Technique.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/UI/Text3D.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/IO/Log.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/metadata.h>
#include <assimp/postprocess.h>

#include "JumpPad.h"
#include "Ladder.h"

// #include <iostream>
#include <vector>
#include <string>
#include <string_view>
#include <cstring>

#ifdef USING_RBFX
typedef ea::string String;
#endif // USING_RBFX

using namespace Urho3D;

static void processAssimpLights(const aiScene* ai_scene, Node* parentNode)
{
    for (unsigned int i = 0; i < ai_scene->mNumLights; ++i)
    {
        aiLight* ai_light = ai_scene->mLights[i];
        Node *realParentNode = parentNode->GetChild(ai_light->mName.C_Str(), true);
        if (!realParentNode)
            realParentNode = parentNode;
        Node* lightNode = realParentNode->CreateChild(ai_light->mName.C_Str());
        Light* light = lightNode->CreateComponent<Light>();

        if (ai_light->mType == aiLightSource_POINT)
        {
            light->SetLightType(LIGHT_POINT);
            light->SetRange(ai_light->mAttenuationLinear * 10.0f); // Example scaling
        } else if (ai_light->mType == aiLightSource_DIRECTIONAL)
        {
            light->SetLightType(LIGHT_DIRECTIONAL);
        } else if (ai_light->mType == aiLightSource_SPOT)
        {
            light->SetLightType(LIGHT_SPOT);
            light->SetFov(Urho3D::ToDegrees(ai_light->mAngleOuterCone));
        }

        // set position
        if (ai_light->mType != aiLightSource_DIRECTIONAL)
            lightNode->SetPosition(Vector3(ai_light->mPosition.x, ai_light->mPosition.y, ai_light->mPosition.z));

        // TODO set color
        // std::cout << "COLOR: (" << ai_light->mColorDiffuse.r << "," << ai_light->mColorDiffuse.g << "," << ai_light->mColorDiffuse.b << ")" << std::endl;
        // light->SetColor(Color(ai_light->mColorDiffuse.r, ai_light->mColorDiffuse.g, ai_light->mColorDiffuse.b));
        light->SetColor(Color(1.0f, 1.0f, 1.0f));

        // TODO set range & brightness
        light->SetRange(50.0f);
        light->SetBrightness(1.0f);

        // enable shadow casting
        light->SetCastShadows(true);
#ifdef USING_RBFX
        light->SetShadowBias(BiasParameters(0.000025f, 1.0f, 0.001));
#else // U3D
        light->SetShadowBias(BiasParameters(0.000025f, 0.5f, 0.001));
#endif // USING_RBFX
        light->SetShadowCascade(CascadeParameters(10.0f, 50.0f, 200.0f, 0.0f, 0.8f));
    }
}

static SharedPtr<Model> loadModel(const aiMesh * const ai_mesh, Context * const context)
{
    SharedPtr<Model> model(new Model(context));
    SharedPtr<VertexBuffer> vb(new VertexBuffer(context));
    SharedPtr<IndexBuffer> ib(new IndexBuffer(context));
    SharedPtr<Geometry> geom(new Geometry(context));

    // enable keeping a CPU-side copy of the data, needed later for physics
    vb->SetShadowed(true);
    ib->SetShadowed(true);

    // Vertex buffer
    unsigned vertexCount = ai_mesh->mNumVertices;
    vb->SetSize(vertexCount, MASK_POSITION | MASK_NORMAL | MASK_TEXCOORD1 | MASK_TANGENT); // Adjust mask as needed
    std::vector<float> vertexData;
    vertexData.reserve(vertexCount * 12); // P(3) + N(3) + T(2) + Tangent(4) = 12 floats

    for (unsigned j = 0; j < vertexCount; ++j)
    {
        vertexData.push_back(ai_mesh->mVertices[j].x);
        vertexData.push_back(ai_mesh->mVertices[j].y);
        vertexData.push_back(ai_mesh->mVertices[j].z);

        if (ai_mesh->HasNormals())
        {
            vertexData.push_back(ai_mesh->mNormals[j].x);
            vertexData.push_back(ai_mesh->mNormals[j].y);
            vertexData.push_back(ai_mesh->mNormals[j].z);
        } else {
            vertexData.push_back(0.0f);
            vertexData.push_back(1.0f);
            vertexData.push_back(0.0f);
        }

        if (ai_mesh->HasTextureCoords(0))
        {
            vertexData.push_back(ai_mesh->mTextureCoords[0][j].x);
            vertexData.push_back(ai_mesh->mTextureCoords[0][j].y);
        } else {
            vertexData.push_back(0.0f);
            vertexData.push_back(0.0f);
        }

        if (ai_mesh->HasTangentsAndBitangents())
        {
            vertexData.push_back(ai_mesh->mTangents[j].x);
            vertexData.push_back(ai_mesh->mTangents[j].y);
            vertexData.push_back(ai_mesh->mTangents[j].z);
            vertexData.push_back(1.0f); // W component for tangent
        } else {
            vertexData.push_back(1.0f);
            vertexData.push_back(0.0f);
            vertexData.push_back(0.0f);
            vertexData.push_back(1.0f);
        }
    }
    // Changed Lock/Unlock to SetData
#ifdef USING_RBFX
    vb->Update(vertexData.data());
#else // U3D
    vb->SetData(vertexData.data());
#endif

    // Index buffer
    unsigned indexCount = ai_mesh->mNumFaces * 3;
    ib->SetSize(indexCount, true); // true for 32-bit indices, false for 16-bit indices?
    std::vector<uint32_t> indexData;
    indexData.reserve(indexCount);
    for (unsigned j = 0; j < ai_mesh->mNumFaces; ++j)
    {
        aiFace& face = ai_mesh->mFaces[j];
        indexData.push_back(face.mIndices[0]);
        indexData.push_back(face.mIndices[1]);
        indexData.push_back(face.mIndices[2]);
    }
    // Changed Lock/Unlock to SetData
#ifdef USING_RBFX
    ib->Update(indexData.data());
#else // U3D
    ib->SetData(indexData.data());
#endif

    geom->SetVertexBuffer(0, vb);
    geom->SetIndexBuffer(ib);
    geom->SetDrawRange(TRIANGLE_LIST, 0, indexCount);

    model->SetNumGeometries(1);
    model->SetGeometry(0, 0, geom);

    // Set bounding box
    BoundingBox bb;
    for (unsigned j = 0; j < vertexCount; ++j)
    {
        Vector3 pos(ai_mesh->mVertices[j].x, ai_mesh->mVertices[j].y, ai_mesh->mVertices[j].z);
        bb.Merge(pos);
    }
    model->SetBoundingBox(bb);

    return model;
}

static Node* AddText3DLabel(Node* targetNode, const String& text, const Color& color = Color::WHITE, float offsetY = 2.5f, float fontSize = 24.0f)
{
    Context* context = targetNode->GetContext();
    ResourceCache* cache = context->GetSubsystem<ResourceCache>();

    // Create a child node for the label
    Node* labelNode = targetNode->CreateChild("Text3DLabel");

    // Position above the target node
    Vector3 position = Vector3::ZERO;
    StaticModel* sm = targetNode->GetComponent<StaticModel>();
    if (sm && sm->GetModel())
    {
        BoundingBox bb = sm->GetModel()->GetBoundingBox();
        position.y_ = bb.max_.y_ * targetNode->GetScale().y_ + offsetY; // Above bounding box // TODO I think this isn't quite right...
    }
    else
    {
        position.y_ = offsetY; // Fallback fixed offset
    }
    labelNode->SetPosition(position);
    labelNode->SetWorldScale(Vector3::ONE);

    // Create Text3D component
    Text3D* text3D = labelNode->CreateComponent<Text3D>();

    // Load font (use default Urho3D font or custom)
    Font *font = cache->GetResource<Font>("Fonts/Anonymous Pro.ttf");
    if (!font)
    {
        // URHO3D_LOGWARNING("Failed to load font 'Fonts/Anonymous Pro.ttf', using fallback");
        font = cache->GetResource<Font>("Fonts/BlueHighway.ttf"); // Fallback font
    }

    // Configure Text3D properties
    text3D->SetText(text);
    text3D->SetFont(font, fontSize);
    text3D->SetColor(color);
    text3D->SetTextAlignment(HA_CENTER); // Center horizontally
    text3D->SetHorizontalAlignment(HA_CENTER); // Ensure 3D alignment
    text3D->SetVerticalAlignment(VA_CENTER);
    text3D->SetFaceCameraMode(FC_ROTATE_XYZ); // Billboard mode
    text3D->SetFixedScreenSize(true);
    // text3D->SetRelative(true); // doesn't exist :-(
    text3D->SetCastShadows(false); // Optional: disable shadows for clarity

    // URHO3D_LOGINFOF("Added Text3D label '%s' to node '%s' at offset y=%.3f",
        // text.CString(), targetNode->GetName().CString(), position.y_);

    return labelNode;
}

float ReadNumber(const aiMetadataEntry * const entry, bool *ok = nullptr)
{
    bool resultOk = true;
    float result = 0.0f;
    switch (entry->mType)
    {
        case AI_INT32: result = *static_cast<const int32_t *>(entry->mData); break;
        case AI_UINT64: result = *static_cast<const uint64_t *>(entry->mData); break;
        case AI_FLOAT: result = *static_cast<const float *>(entry->mData); break;
        case AI_DOUBLE: result = *static_cast<const double *>(entry->mData); break;
        // case AI_INT64: result = *static_cast<const int64_t *>(entry->mData); break;
        // case AI_UINT32: result = *static_cast<const uint32_t *>(entry->mData); break;
        default:
        resultOk = false;
        break;
    }
    if (ok)
        *ok = resultOk;
    return result;
}

static void processAssimpNode(const aiNode* ai_node, const aiScene* ai_scene, Node* parentNode, Context *context)
{
    /*
    Node* currentNode = parent->CreateChild(ai_node->mName.C_Str());

    // Set transform
    aiMatrix4x4 t = ai_node->mTransformation;
    Vector3 pos(t.a4, t.b4, t.c4);
    Quaternion rot;
    Vector3 scale;
    Matrix3x4(mat3x4).Decompose(pos, rot, scale);
    currentNode->SetPosition(pos);
    currentNode->SetRotation(rot);
    currentNode->SetScale(scale);
    */

    // create the node
    Node* currentNode = parentNode->CreateChild(ai_node->mName.C_Str());

    // apply the transformation
    {
        aiMatrix4x4 transform = ai_node->mTransformation;
        aiVector3t<float> scaling, position;
        aiQuaterniont<float> rotation;
        transform.Decompose(scaling, rotation, position);

        Vector3 scale(scaling.x, scaling.y, scaling.z);
        Vector3 translate(position.x, position.y, position.z);
        Quaternion orient(rotation.w, rotation.x, rotation.y, rotation.z);

        currentNode->SetPosition(translate);
        currentNode->SetRotation(orient);
        currentNode->SetScale(scale);
    }

    float rigidBodyMass = 0.0f;
    if (ai_node->mMetaData)
    {
        for (unsigned int i = 0; i < ai_node->mMetaData->mNumProperties; i++)
        {
            // std::cout << "property #" << i << ": \"" << ai_node->mMetaData->mKeys[i].C_Str() << "\": type " << ai_node->mMetaData->mValues[i].mType << "" << std::endl;
            // find extensions entry
            if (ai_node->mMetaData->mValues[i].mType == AI_AIMETADATA && strcmp(ai_node->mMetaData->mKeys[i].C_Str(), "extensions") == 0)
            {
                // std::cout << "Found \"extensions\"!" << std::endl;
                const aiMetadata * const extensions = static_cast<const aiMetadata *>(ai_node->mMetaData->mValues[i].mData);
                for (unsigned int j = 0; j < extensions->mNumProperties; j++)
                {
                    // std::cout << "\tproperty #" << j << ": \"" << extensions->mKeys[j].C_Str() << "\": type " << extensions->mValues[j].mType << "" << std::endl;
                    // find KHR_physics_rigid_bodies entry
                    if (extensions->mValues[j].mType == AI_AIMETADATA && strcmp(extensions->mKeys[j].C_Str(), "KHR_physics_rigid_bodies") == 0)
                    {
                        // std::cout << "\tFound \"KHR_physics_rigid_bodies\"!" << std::endl;
                        const aiMetadata * const rigid_body_metadata = static_cast<const aiMetadata *>(extensions->mValues[j].mData);
                        for (unsigned int k = 0; k < rigid_body_metadata->mNumProperties; k++)
                        {
                            // std::cout << "\t\tproperty #" << k << ": \"" << rigid_body_metadata->mKeys[k].C_Str() << "\": type " << rigid_body_metadata->mValues[k].mType << "" << std::endl;
                            // find motion entry
                            if (rigid_body_metadata->mValues[k].mType == AI_AIMETADATA && strcmp(rigid_body_metadata->mKeys[k].C_Str(), "motion") == 0)
                            {
                                // std::cout << "\t\tFound \"motion\"!" << std::endl;
                                const aiMetadata * const motion_metadata = static_cast<const aiMetadata *>(rigid_body_metadata->mValues[k].mData);
                                for (unsigned int l = 0; l < motion_metadata->mNumProperties; l++)
                                {
                                    // std::cout << "\t\t\tproperty #" << l << ": \"" << motion_metadata->mKeys[l].C_Str() << "\": type " << motion_metadata->mValues[l].mType << "" << std::endl;
                                    if (strcmp(motion_metadata->mKeys[l].C_Str(), "mass") == 0)
                                    {
                                        // std::cout << "\t\t\tFound \"mass\"!" << std::endl;
                                        bool ok = false;
                                        rigidBodyMass = ReadNumber(motion_metadata->mValues + l, &ok);
                                        if (!ok)
                                        {
                                            // std::cout << "UNKNOWN TYPE! " << motion_metadata->mValues[l].mType << std::endl;
                                            continue;
                                        }
                                        // std::cout << "\t\t\t\tmass: " << mass << std::endl;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // TODO cache models
    // std::vector<SharedPtr<Model>> models(ai_node->mNumMeshes);

    for (unsigned int i = 0; i < ai_node->mNumMeshes; ++i)
    {
        const auto meshIndex = ai_node->mMeshes[i];
        const aiMesh * const ai_mesh = ai_scene->mMeshes[meshIndex];

        // load mesh
        SharedPtr<Model> model = loadModel(ai_mesh, context);
        // models[i] = model;

        // apply mesh
        StaticModel* sm = currentNode->CreateComponent<StaticModel>();
        sm->SetModel(model);
        sm->SetCastShadows(true);

        // apply material
        const auto materialIndex = ai_mesh->mMaterialIndex;
        if (materialIndex < ai_scene->mNumMaterials)
        {
            aiMaterial * const ai_mat = ai_scene->mMaterials[materialIndex];
            aiColor4D diffuseColor;
            if (AI_SUCCESS != aiGetMaterialColor(ai_mat, AI_MATKEY_BASE_COLOR, &diffuseColor))
            {
                // Fall back to diffuse if base color isn't set
                aiGetMaterialColor(ai_mat, AI_MATKEY_COLOR_DIFFUSE, &diffuseColor);
            }

            SharedPtr<Material> mat(new Material(context));
            mat->SetTechnique(0, context->GetSubsystem<ResourceCache>()->GetResource<Technique>("Techniques/NoTextureAO.xml"));
            mat->SetShaderParameter("MatDiffColor", Color(diffuseColor.r, diffuseColor.g, diffuseColor.b));
            // mat->SetShadowCullMode(CULL_CW);

            sm->SetMaterial(mat);
        }

        // create physics body
        RigidBody * const body = currentNode->CreateComponent<RigidBody>();
        body->SetMass(rigidBodyMass); // defaults to 0.0 which means a static body

        // create physics shape
        CollisionShape * const shape = currentNode->CreateComponent<CollisionShape>();
        if (rigidBodyMass == 0.0f)
            shape->SetTriangleMesh(model); // for static bodies, we can use non-convex geometry
        else
            shape->SetConvexHull(model); // for dynamic bodies, the geometry must be convex!
    }

    // check for custom game object type
    if (const aiMetadata * const metadata = ai_node->mMetaData)
    {
        for (unsigned int i = 0; i < metadata->mNumProperties; ++i)
        {
            const std::string_view key = metadata->mKeys[i].C_Str();
            const aiMetadataEntry &entry = metadata->mValues[i];

            if (key == "GameObjectType" && entry.mType == AI_AISTRING)
            {
                const aiString * const val = static_cast<const aiString*>(entry.mData);
                const char * const type = val->C_Str();
                // TODO store pointers, we are leaking these object currently!
                if (strcmp(type, "JumpPad") == 0)
                {
                    JumpPad * const jumpPad = new JumpPad(currentNode);
                }
                else if (strcmp(type, "Ladder") == 0)
                {
                    Ladder * const ladder = new Ladder(currentNode);
                }
            }
        }
    }

    AddText3DLabel(currentNode, ai_node->mName.C_Str());

    // recursively process children
    for (unsigned int i = 0; i < ai_node->mNumChildren; ++i)
        processAssimpNode(ai_node->mChildren[i], ai_scene, currentNode, context);
}

void loadSceneWithAssimp(const std::string& filename, Node* parentNode, Context* context)
{
    Assimp::Importer importer;
    const aiScene* ai_scene = importer.ReadFile(filename,
        aiProcess_Triangulate |
        aiProcess_GenSmoothNormals |
        aiProcess_JoinIdenticalVertices |
        aiProcess_ImproveCacheLocality |
        aiProcess_RemoveRedundantMaterials |
        aiProcess_SortByPType// |
        //aiProcess_PreTransformVertices
    );

    if (!ai_scene || !ai_scene->mRootNode)
    {
        // std::cerr << "Error loading scene: " << importer.GetErrorString() << std::endl;
        return;
    }

    processAssimpNode(ai_scene->mRootNode, ai_scene, parentNode, context);
    processAssimpLights(ai_scene, parentNode);
}
