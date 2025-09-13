#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Engine/Application.h>
#include <Urho3D/Engine/EngineDefs.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/CustomGeometry.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/GraphicsEvents.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/Technique.h>
#include <Urho3D/Graphics/VertexBuffer.h>
#include <Urho3D/Graphics/Viewport.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/Input/InputEvents.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#ifdef USING_RBFX
#include <Urho3D/RenderPipeline/RenderPipeline.h>
#include <Urho3D/RenderPipeline/ShaderConsts.h>
#endif // USING_RBFX
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#ifdef USING_RBFX
#include <Urho3D/SystemUI/DebugHud.h>
#else // USING_RBFX
#include <Urho3D/Engine/DebugHud.h>
#endif // USING_RBFX
#include <Urho3D/IO/Log.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/UI.h>
#include <Urho3D/UI/UIEvents.h>

#include "SceneLoader.h"

using namespace Urho3D;

#ifndef USING_RBFX
namespace ea {

template <typename T>
class vector : public Urho3D::PODVector<T>
{
public:
    void push_back(const T &value) {Urho3D::PODVector<T>::Push(value);}
    // void push_back(const T &&value) {Urho3D::PODVector<T>::Push(value);}
    std::size_t size() const {return Urho3D::PODVector<T>::Size();}
    T * data() {return Urho3D::PODVector<T>::Buffer();}
    const T * data() const {return const_cast<const T*>(Urho3D::PODVector<T>::Buffer());}
};

} // namespace ea
// template <typename T>
// typedef ea::vector<T> Urho3D::PODVector<T>;
#endif // USING_RBFX

class MyApp : public Application
{
    URHO3D_OBJECT(MyApp, Application)

public:
    MyApp(Context* context) : Application(context), yaw_(0.0f), pitch_(0.0f), drawDebug_(false) {}

    virtual void Setup() override
    {
        engineParameters_[EP_FULL_SCREEN] = false;
        engineParameters_[EP_WINDOW_WIDTH] = 1280;
        engineParameters_[EP_WINDOW_HEIGHT] = 720;
        engineParameters_[EP_WINDOW_RESIZABLE] = true;
        engineParameters_[EP_BORDERLESS] = false;
        engineParameters_[EP_VSYNC] = true;
    }

    virtual void Start() override
    {
        ResourceCache* cache = GetSubsystem<ResourceCache>();

        // Create scene
        scene_ = new Scene(context_);
        octree_ = scene_->CreateComponent<Octree>();
        physicsWorld_ = scene_->CreateComponent<PhysicsWorld>();
        physicsWorld_->SetMaxSubSteps(10);
        physicsWorld_->SetFps(240);
        DebugRenderer * const debugRenderer = scene_->CreateComponent<DebugRenderer>();
        zone_ = scene_->CreateComponent<Zone>();
        zone_->SetBoundingBox(BoundingBox(-1000.0f, 1000.0f));
        zone_->SetAmbientColor(Color(0.1f, 0.1f, 0.1f));
        zone_->SetFogColor(Color(0.5f, 0.5f, 0.7f));
        zone_->SetFogStart(100.0f);
        zone_->SetFogEnd(300.0f);

#ifdef USING_RBFX
        {
            RenderPipeline * const renderPipeline = scene_->CreateComponent<RenderPipeline>();
            RenderPipelineSettings settings = renderPipeline->GetSettings();
            settings.renderBufferManager_.readableDepth_ = true;
            // settings.sceneProcessor_.directionalShadowSize_ = 2048;
            // settings.sceneProcessor_.spotShadowSize_ = 2048;
            settings.sceneProcessor_.pointShadowSize_ = 1024;
            // settings.shadowMapAllocator_.shadowAtlasPageSize_ = 8192;
            // TODO how to set the shadow map quality to 16-bit vs 32-bit?
            renderPipeline->SetSettings(settings);
            renderPipeline->SetRenderPassEnabled(eastl::string("Postprocess: SSAO"), true);
        }
#endif // USING_RBFX

        // Create procedural plane model for floor
        /*CreateFloor();

        // Create cubes
        CreateCube(Vector3(-2.0f, 0.5f, 0.0f), Color(1.0f, 0.0f, 0.0f));
        CreateCube(Vector3(2.0f, 0.5f, 0.0f), Color(0.0f, 1.0f, 0.0f));
        CreateCube(Vector3(0.0f, 0.5f, 2.0f), Color(0.0f, 0.0f, 1.0f));
        // Stacked cubes
        CreateCube(Vector3(0.0f, 0.5f, -2.0f), Color(1.0f, 1.0f, 0.0f));
        CreateCube(Vector3(0.0f, 1.5f, -2.0f), Color(0.0f, 1.0f, 1.0f));

        // Point light
        Node* lightNode = scene_->CreateChild("PointLight");
        lightNode->SetPosition(Vector3(5.0f, 3.0f, 5.0f));
        Light* light = lightNode->CreateComponent<Light>();
        light->SetLightType(LIGHT_POINT);
        light->SetRange(50.0f);
        light->SetBrightness(1.5f);
        light->SetColor(Color(1.0f, 1.0f, 1.0f));
        light->SetCastShadows(true);
        // light->SetShadowResolution(2048);
        light->SetShadowBias(BiasParameters(0.0001f, -0.1f, 0.001));
        light->SetShadowCascade(CascadeParameters(10.0f, 50.0f, 200.0f, 0.0f, 0.8f));*/
        
        loadSceneWithAssimp("../assets/test_scene_torus.glb", scene_, context_);

        // Camera
        cameraNode_ = scene_->CreateChild("Camera");
        cameraNode_->SetPosition(Vector3(0.0f, 5.0f, -20.0f));
        camera_ = cameraNode_->CreateComponent<Camera>();
        camera_->SetFarClip(300.0f);

        // Viewport
        Renderer* renderer = GetSubsystem<Renderer>();
        SharedPtr<Viewport> viewport(new Viewport(context_, scene_, camera_));
        renderer->SetViewport(0, viewport);

        // Debug HUD for FPS
        debugHud_ = engine_->CreateDebugHud();
#ifndef USING_RBFX
        debugHud_->SetDefaultStyle(cache->GetResource<XMLFile>("UI/DefaultStyle.xml"));
        debugHud_->SetMode(DEBUGHUD_SHOW_ALL);
        // Font* font = cache->GetResource<Font>("Fonts/Anonymous Pro.ttf");
#endif // USING_RBFX

        // Set mouse mode for FPS control
        Input* input = GetSubsystem<Input>();
        input->SetMouseVisible(false);
        input->SetMouseMode(MM_RELATIVE);

        // Subscribe to events
        SubscribeToEvent(E_KEYDOWN, URHO3D_HANDLER(MyApp, HandleKeyDown));
        SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(MyApp, HandleUpdate));
        SubscribeToEvent(E_MOUSEMOVE, URHO3D_HANDLER(MyApp, HandleMouseMove));
        // SubscribeToEvent(E_PHYSICSCOLLISION, URHO3D_HANDLER(MyApp, HandlePhysicsCollision));
        SubscribeToEvent(E_POSTRENDERUPDATE, URHO3D_HANDLER(MyApp, HandlePostRenderUpdate));
    }

    virtual void Stop() override {}

    void HandleKeyDown(StringHash eventType, VariantMap& eventData)
    {
        int key = eventData[KeyDown::P_KEY].GetInt();
        if (key == KEY_ESCAPE) {
            engine_->Exit();
        }
    }

    void HandleUpdate(StringHash eventType, VariantMap& eventData)
    {
        float timeStep = eventData[Update::P_TIMESTEP].GetFloat();
        Input* input = GetSubsystem<Input>();

        // Movement
        float moveSpeed = 10.0f;
        if (input->GetKeyDown(KEY_W)) cameraNode_->Translate(Vector3::FORWARD * moveSpeed * timeStep);
        if (input->GetKeyDown(KEY_S)) cameraNode_->Translate(Vector3::BACK * moveSpeed * timeStep);
        if (input->GetKeyDown(KEY_A)) cameraNode_->Translate(Vector3::LEFT * moveSpeed * timeStep);
        if (input->GetKeyDown(KEY_D)) cameraNode_->Translate(Vector3::RIGHT * moveSpeed * timeStep);
        if (input->GetKeyPress(KEY_X))
            camera_->SetFillMode(camera_->GetFillMode() == FILL_WIREFRAME ? FILL_SOLID : FILL_WIREFRAME);
        if (input->GetKeyPress(KEY_SPACE))
            drawDebug_ = !drawDebug_;

        // shoot sphere on left mouse click
        if (input->GetMouseButtonPress(MOUSEB_LEFT))
        {
            // Graphical representation
            Node *sphereNode = CreateSphere(cameraNode_->GetWorldPosition(), Color(1.0f, 1.0f, 1.0f));
            // AddText3DLabel(sphereNode, "Ball");

            // Physics representation
            RigidBody* body = sphereNode->CreateComponent<RigidBody>();
            body->SetMass(1.0f);
            body->SetFriction(0.5f);
            body->SetLinearDamping(0.0f);
            body->SetAngularDamping(0.2f);
            CollisionShape* shape = sphereNode->CreateComponent<CollisionShape>();
            shape->SetSphere(0.5f); // Diameter of 0.5f (radius 0.25f)

            // Shoot in camera direction
            Vector3 direction = cameraNode_->GetWorldDirection().Normalized();
            float speed = 25.0f;
            body->SetLinearVelocity(direction * speed);
        }

        // Update debug HUD (shows FPS)
        debugHud_->SetMode(DEBUGHUD_SHOW_ALL);
    }

    void HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData)
    {
        // If draw debug mode is enabled, draw viewport debug geometry. Disable depth test so that we can see the effect of occlusion
        if (drawDebug_)
        {
            // GetSubsystem<Renderer>()->DrawDebugGeometry(false);
            physicsWorld_->DrawDebugGeometry(true);
        }
    }

    void HandleMouseMove(StringHash eventType, VariantMap& eventData)
    {
        Input* input = GetSubsystem<Input>();
        if (input->GetMouseMode() != MM_RELATIVE) return;

        int dx = eventData[MouseMove::P_DX].GetInt();
        int dy = eventData[MouseMove::P_DY].GetInt();

        const float mouseSensitivity = 0.2f;
        yaw_ += dx * mouseSensitivity;
        pitch_ += dy * mouseSensitivity;
        pitch_ = Clamp(pitch_, -90.0f, 90.0f);

        cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));
    }

    void HandlePhysicsCollision(StringHash eventType, VariantMap& eventData)
    {
        Node* nodeA = static_cast<Node*>(eventData[PhysicsCollision::P_NODEA].GetPtr());
        Node* nodeB = static_cast<Node*>(eventData[PhysicsCollision::P_NODEB].GetPtr());
        RigidBody* bodyA = static_cast<RigidBody*>(eventData[PhysicsCollision::P_BODYA].GetPtr());
        RigidBody* bodyB = static_cast<RigidBody*>(eventData[PhysicsCollision::P_BODYB].GetPtr());
        VectorBuffer contacts = eventData[PhysicsCollision::P_CONTACTS].GetVectorBuffer();

        // Process each contact point
        while (!contacts.IsEof())
        {
            Vector3 position = contacts.ReadVector3();
            Vector3 normal = contacts.ReadVector3();
            float distance = contacts.ReadFloat();
            float impulse = contacts.ReadFloat();

            // Log contact details (similar to gContactProcessedCallback)
#ifdef USING_RBFX // TODO: use alternative string methods
            URHO3D_LOGINFOF("Collision between %s and %s: Impulse=%.2f, Position=(%.2f, %.2f, %.2f)",
                            nodeA->GetName().c_str(), nodeB->GetName().c_str(),
                            impulse, position.x_, position.y_, position.z_);
#endif // USING_RBFX

            // Example: Play sound or modify contact properties based on impulse
            if (impulse > 0.1f) // Threshold for significant impact
            {
                // Add custom logic, e.g., play sound, spawn particles
            }
        }
    }

private:
    void CreateFloor()
    {
        // Create floor plane
        Node* floorNode = scene_->CreateChild("Floor");
        floorNode->SetPosition(Vector3(0.0f, 0.0f, 0.0f));
        CustomGeometry* floorGeom = floorNode->CreateComponent<CustomGeometry>();
        floorGeom->BeginGeometry(0, TRIANGLE_LIST);
        // Define square plane (two triangles)
        floorGeom->DefineVertex(Vector3(-10.0f, 0.0f, 10.0f)); floorGeom->DefineNormal(Vector3::UP); floorGeom->DefineTexCoord(Vector2(0.0f, 1.0f));
        floorGeom->DefineVertex(Vector3(10.0f, 0.0f, 10.0f)); floorGeom->DefineNormal(Vector3::UP); floorGeom->DefineTexCoord(Vector2(1.0f, 1.0f));
        floorGeom->DefineVertex(Vector3(-10.0f, 0.0f, -10.0f)); floorGeom->DefineNormal(Vector3::UP); floorGeom->DefineTexCoord(Vector2(0.0f, 0.0f));
        floorGeom->DefineVertex(Vector3(10.0f, 0.0f, 10.0f)); floorGeom->DefineNormal(Vector3::UP); floorGeom->DefineTexCoord(Vector2(1.0f, 1.0f));
        floorGeom->DefineVertex(Vector3(10.0f, 0.0f, -10.0f)); floorGeom->DefineNormal(Vector3::UP); floorGeom->DefineTexCoord(Vector2(1.0f, 0.0f));
        floorGeom->DefineVertex(Vector3(-10.0f, 0.0f, -10.0f)); floorGeom->DefineNormal(Vector3::UP); floorGeom->DefineTexCoord(Vector2(0.0f, 0.0f));
        floorGeom->Commit();
        floorGeom->SetMaterial(CreateMaterial(Color(0.8f, 0.8f, 0.8f)));
        floorGeom->SetCastShadows(false);  // Floor doesn't cast shadows

        RigidBody* floorBody = floorNode->CreateComponent<RigidBody>();
        floorBody->SetMass(0.0f); // Static body
        CollisionShape* floorShape = floorNode->CreateComponent<CollisionShape>();
        floorShape->SetBox(Vector3(20.0f, 1.0f, 20.0f), Vector3(0.0f, -0.5f, 0.0f));
        // floorShape->SetTriangleMesh(floorGeom->GetLodGeometry(0, 0));
    }

    void CreateCube(const Vector3& pos, const Color& color)
    {
        Node* cubeNode = scene_->CreateChild("Cube");
        cubeNode->SetPosition(pos);
        cubeNode->SetScale(Vector3(1.0f, 1.0f, 1.0f));
        CustomGeometry* cubeGeom = cubeNode->CreateComponent<CustomGeometry>();
        cubeGeom->BeginGeometry(0, TRIANGLE_LIST);

        // Define cube vertices (positions, normals; no texcoords needed)
        // Rear face (Z = -0.5, outward normal BACK)
        cubeGeom->DefineVertex(Vector3(0.5f, 0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::BACK);
        cubeGeom->DefineVertex(Vector3(0.5f, -0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::BACK);
        cubeGeom->DefineVertex(Vector3(-0.5f, -0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::BACK);
        cubeGeom->DefineVertex(Vector3(-0.5f, 0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::BACK);
        cubeGeom->DefineVertex(Vector3(0.5f, 0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::BACK);
        cubeGeom->DefineVertex(Vector3(-0.5f, -0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::BACK);
        // Front face (Z = 0.5, outward normal FORWARD)
        cubeGeom->DefineVertex(Vector3(0.5f, -0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::FORWARD);
        cubeGeom->DefineVertex(Vector3(0.5f, 0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::FORWARD);
        cubeGeom->DefineVertex(Vector3(-0.5f, 0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::FORWARD);
        cubeGeom->DefineVertex(Vector3(-0.5f, -0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::FORWARD);
        cubeGeom->DefineVertex(Vector3(0.5f, -0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::FORWARD);
        cubeGeom->DefineVertex(Vector3(-0.5f, 0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::FORWARD);
        // Left face (X = -0.5, outward normal LEFT)
        cubeGeom->DefineVertex(Vector3(-0.5f, 0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::LEFT);
        cubeGeom->DefineVertex(Vector3(-0.5f, 0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::LEFT);
        cubeGeom->DefineVertex(Vector3(-0.5f, -0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::LEFT);
        cubeGeom->DefineVertex(Vector3(-0.5f, -0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::LEFT);
        cubeGeom->DefineVertex(Vector3(-0.5f, 0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::LEFT);
        cubeGeom->DefineVertex(Vector3(-0.5f, -0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::LEFT);
        // Right face (X = 0.5, outward normal RIGHT)
        cubeGeom->DefineVertex(Vector3(0.5f, 0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::RIGHT);
        cubeGeom->DefineVertex(Vector3(0.5f, 0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::RIGHT);
        cubeGeom->DefineVertex(Vector3(0.5f, -0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::RIGHT);
        cubeGeom->DefineVertex(Vector3(0.5f, -0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::RIGHT);
        cubeGeom->DefineVertex(Vector3(0.5f, 0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::RIGHT);
        cubeGeom->DefineVertex(Vector3(0.5f, -0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::RIGHT);
        // Bottom face (Y = -0.5, outward normal DOWN)
        cubeGeom->DefineVertex(Vector3(0.5f, -0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::DOWN);
        cubeGeom->DefineVertex(Vector3(-0.5f, -0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::DOWN);
        cubeGeom->DefineVertex(Vector3(-0.5f, -0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::DOWN);
        cubeGeom->DefineVertex(Vector3(0.5f, -0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::DOWN);
        cubeGeom->DefineVertex(Vector3(-0.5f, -0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::DOWN);
        cubeGeom->DefineVertex(Vector3(0.5f, -0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::DOWN);
        // Top face (Y = 0.5, outward normal UP)
        cubeGeom->DefineVertex(Vector3(-0.5f, 0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::UP);
        cubeGeom->DefineVertex(Vector3(0.5f, 0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::UP);
        cubeGeom->DefineVertex(Vector3(-0.5f, 0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::UP);
        cubeGeom->DefineVertex(Vector3(-0.5f, 0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::UP);
        cubeGeom->DefineVertex(Vector3(0.5f, 0.5f, 0.5f)); cubeGeom->DefineNormal(Vector3::UP);
        cubeGeom->DefineVertex(Vector3(0.5f, 0.5f, -0.5f)); cubeGeom->DefineNormal(Vector3::UP);

        cubeGeom->Commit();
        cubeGeom->SetMaterial(CreateMaterial(color));
        cubeGeom->SetCastShadows(true);

        RigidBody* cubeBody = cubeNode->CreateComponent<RigidBody>();
        cubeBody->SetMass(1.0f); // Dynamic body
        cubeBody->SetFriction(0.5f);
        CollisionShape* cubeShape = cubeNode->CreateComponent<CollisionShape>();
        cubeShape->SetBox(Vector3(1.0f, 1.0f, 1.0f));
        // cubeShape->SetBox(Vector3(1.0f, 1.0f, 1.0f), pos);
    }

    Node * CreateSphere(const Vector3& pos, const Color& color)
    {
        Node* sphereNode = scene_->CreateChild("Sphere");
        sphereNode->SetPosition(pos);
        sphereNode->SetScale(Vector3(1.0f, 1.0f, 1.0f));

        // create a cached model
        if (!sphereModel_)
            sphereModel_ = CreateSphereModel();

        StaticModel* sm = sphereNode->CreateComponent<StaticModel>();
        sm->SetModel(sphereModel_);
        sm->SetMaterial(CreateMaterial(color));
        sm->SetCastShadows(true);

        return sphereNode;
    }

    SharedPtr<Model> CreateSphereModel(float radius = 0.25f, int stacks = 16, int slices = 16)
    {
        SharedPtr<Model> model(new Model(context_));
        SharedPtr<VertexBuffer> vb(new VertexBuffer(context_));
        SharedPtr<IndexBuffer> ib(new IndexBuffer(context_));
        SharedPtr<Geometry> geom(new Geometry(context_));

        // Enable CPU-side data for physics
        vb->SetShadowed(true);
        ib->SetShadowed(true);

        // Calculate vertices, normals, and texture coordinates
        ea::vector<Vector3> vertices;
        ea::vector<Vector3> normals;
        // PODVector<Vector2> texCoords;
        for (int stack = 0; stack <= stacks; ++stack)
        {
            const float phi = static_cast<float>(stack) * 180.0f / stacks;
            const float sinPhi = Sin(phi);
            const float cosPhi = Cos(phi);
            // const float v = static_cast<float>(stack) / stacks; // Texture V coordinate

            for (int slice = 0; slice <= slices; ++slice)
            {
                const float theta = static_cast<float>(slice) * 360.0f / slices;
                // const float u = static_cast<float>(slice) / slices; // Texture U coordinate
                Vector3 vertex = Vector3(sinPhi * Cos(theta), cosPhi, sinPhi * Sin(theta)) * radius;
                vertices.push_back(vertex);
                normals.push_back(vertex.Normalized());
                // texCoords.push_back(Vector2(u, 1.0f - v)); // Flip V for correct orientation
            }
        }

        // Vertex buffer (position, normal, texcoord)
        unsigned vertexCount = vertices.size();
        vb->SetSize(vertexCount, MASK_POSITION | MASK_NORMAL, false); // Static buffer
        ea::vector<float> vertexData;
        for (unsigned i = 0; i < vertexCount; ++i)
        {
            vertexData.push_back(vertices[i].x_);
            vertexData.push_back(vertices[i].y_);
            vertexData.push_back(vertices[i].z_);
            vertexData.push_back(normals[i].x_);
            vertexData.push_back(normals[i].y_);
            vertexData.push_back(normals[i].z_);
            //vertexData.push_back(texCoords[i].x_);
            //vertexData.push_back(texCoords[i].y_);
        }
#ifdef USING_RBFX
        vb->Update(vertexData.data());
#else // USING_RBFX
        vb->SetData(vertexData.data());
#endif // USING_RBFX

        // Index buffer (counterclockwise winding)
        ea::vector<uint16_t> indices;
        for (uint16_t stack = 0; stack < stacks; ++stack)
        {
            const uint16_t topRow = stack * (slices + 1);
            const uint16_t bottomRow = (stack + 1) * (slices + 1);

            for (uint16_t slice = 0; slice < slices; ++slice)
            {
                // First triangle (counterclockwise)
                indices.push_back(topRow + slice);
                indices.push_back(topRow + slice + 1);
                indices.push_back(bottomRow + slice);

                // Second triangle (counterclockwise)
                indices.push_back(topRow + slice + 1);
                indices.push_back(bottomRow + slice + 1);
                indices.push_back(bottomRow + slice);
            }
        }

        const unsigned indexCount = indices.size();
        ib->SetSize(indexCount, false, false); // 16-bit indices, static
#ifdef USING_RBFX
        ib->Update(indices.data());
#else // USING_RBFX
        ib->SetData(indices.data());
#endif // USING_RBFX

        geom->SetVertexBuffer(0, vb);
        geom->SetIndexBuffer(ib);
        geom->SetDrawRange(TRIANGLE_LIST, 0, indexCount);

        model->SetNumGeometries(1);
        model->SetGeometry(0, 0, geom);

        // Set bounding box
        BoundingBox bb;
        bb.Define(Sphere(Vector3::ZERO, 0.25));
        model->SetBoundingBox(bb);

        return model;
    }

    SharedPtr<Material> CreateMaterial(const Color& color)
    {
        ResourceCache* cache = GetSubsystem<ResourceCache>();
        SharedPtr<Material> mat(new Material(context_));
        mat->SetTechnique(0, cache->GetResource<Technique>("Techniques/NoTextureAO.xml"));
#ifdef USING_RBFX
        mat->SetShaderParameter(ShaderConsts::Material_MatDiffColor, color);
#else // USING_RBFX
        mat->SetShaderParameter("MatDiffColor", color);
#endif // USING_RBFX
        mat->SetShadowCullMode(CULL_CW);
        return mat;
    }

    SharedPtr<Scene> scene_;
    SharedPtr<Node> cameraNode_;
    SharedPtr<DebugHud> debugHud_;
    SharedPtr<PhysicsWorld> physicsWorld_;
    SharedPtr<Octree> octree_;
    SharedPtr<Zone> zone_;
    SharedPtr<Camera> camera_;
    SharedPtr<Model> sphereModel_;
    float yaw_;
    float pitch_;
    bool drawDebug_;
};

URHO3D_DEFINE_APPLICATION_MAIN(MyApp);
