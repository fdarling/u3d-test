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
#include <assimp/postprocess.h>

// #include <iostream>
#include <vector>
#include <string>

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
#ifdef USING_RBFX
        light->SetBrightness(3.0f); // Adjust as needed
#else
        light->SetBrightness(1.0f); // Adjust as needed
#endif

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
        RigidBody* body = currentNode->CreateComponent<RigidBody>();
        body->SetMass(0.0f); // Static body
        CollisionShape* shape = currentNode->CreateComponent<CollisionShape>();
        shape->SetTriangleMesh(model);
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
