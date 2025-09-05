#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Engine/Application.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/CustomGeometry.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Technique.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Graphics/Viewport.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/Input/InputEvents.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Engine/DebugHud.h>
#include <Urho3D/Engine/EngineDefs.h>

using namespace Urho3D;

static const char TECHNIQUE_STR[] = "Techniques/NoTexture.xml";

class MyApp : public Application
{
    URHO3D_OBJECT(MyApp, Application);

public:
    MyApp(Context* context) : Application(context), yaw_(0.0f), pitch_(0.0f), drawDebug_(false) {}

    void Setup() override
    {
        engineParameters_[EP_FULL_SCREEN] = false;
        engineParameters_[EP_WINDOW_WIDTH] = 1280;
        engineParameters_[EP_WINDOW_HEIGHT] = 720;
        engineParameters_[EP_RESOURCE_PATHS] = "Data;CoreData";  // Assuming standard U3D resource paths
    }

    void Start() override
    {
        // Subscribe to events
        SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(MyApp, HandleUpdate));
        SubscribeToEvent(E_KEYDOWN, URHO3D_HANDLER(MyApp, HandleKeyDown));
        SubscribeToEvent(E_POSTRENDERUPDATE, URHO3D_HANDLER(MyApp, HandlePostRenderUpdate));

        // Create scene
        scene_ = new Scene(context_);
        scene_->CreateComponent<Octree>();
        DebugRenderer * const debug = scene_->CreateComponent<DebugRenderer>();

        // Create ambient zone
        Node* zoneNode = scene_->CreateChild("Zone");
        Zone* zone = zoneNode->CreateComponent<Zone>();
        zone->SetBoundingBox(BoundingBox(-1000.0f, 1000.0f));
        zone->SetAmbientColor(Color(0.1f, 0.1f, 0.1f));

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
        Material* floorMat = new Material(context_);
        floorMat->SetTechnique(0, GetSubsystem<ResourceCache>()->GetResource<Technique>(TECHNIQUE_STR));
        floorGeom->SetMaterial(floorMat);
        floorGeom->SetCastShadows(false); // Floor doesn't cast shadows

        // Create cubes
        CreateCube(Vector3(-2.0f, 0.5f, 0.0f), Color(1.0f, 0.0f, 0.0f));
        CreateCube(Vector3(2.0f, 0.5f, 0.0f), Color(0.0f, 1.0f, 0.0f));
        CreateCube(Vector3(0.0f, 0.5f, 2.0f), Color(0.0f, 0.0f, 1.0f));
        // Stacked cubes
        CreateCube(Vector3(0.0f, 0.5f, -2.0f), Color(1.0f, 1.0f, 0.0f));
        CreateCube(Vector3(0.0f, 1.5f, -2.0f), Color(0.0f, 1.0f, 1.0f));

        // Create point light
        Node* lightNode = scene_->CreateChild("PointLight");
        lightNode->SetPosition(Vector3(5.0f, 3.0f, 5.0f));
        Light* light = lightNode->CreateComponent<Light>();
        light->SetLightType(LIGHT_POINT);
        light->SetRange(20.0f);
        light->SetBrightness(1.0f);
        light->SetColor(Color(1.0f, 1.0f, 1.0f));
        light->SetCastShadows(true);
        // light->SetShadowBias(BiasParameters(0.00025f, 0.5f));
        // light->SetShadowBias(BiasParameters(0.00001f, 0.5f));
        // light->SetShadowBias(BiasParameters(0.00001f, 0.5f, 0.001)); // seems to work the best!
        light->SetShadowBias(BiasParameters(-0.00001f, -0.5f, 0.001)); // experimental, maybe even better?
        light->SetShadowCascade(CascadeParameters(10.0f, 50.0f, 200.0f, 0.0f, 0.8f));

        // Create camera
        cameraNode_ = scene_->CreateChild("Camera");
        cameraNode_->SetPosition(Vector3(0.0f, 2.0f, -5.0f));
        Camera* camera = cameraNode_->CreateComponent<Camera>();
        camera->SetFarClip(100.0f);

        // Set viewport
        Renderer* renderer = GetSubsystem<Renderer>();
        SharedPtr<Viewport> viewport(new Viewport(context_, scene_, camera));
        renderer->SetViewport(0, viewport);

        // Enable mouse for FPS control
        Input* input = GetSubsystem<Input>();
        input->SetMouseVisible(false);
        input->SetMouseMode(MM_RELATIVE);

        // Create debug HUD for FPS meter (minimal font usage)
        ResourceCache* cache = GetSubsystem<ResourceCache>();
        XMLFile* style = cache->GetResource<XMLFile>("UI/DefaultStyle.xml");
        debugHud_ = engine_->CreateDebugHud();
        debugHud_->SetDefaultStyle(style);
        debugHud_->SetMode(DEBUGHUD_SHOW_ALL);  // Shows FPS, etc.
    }

    void Stop() override
    {
        // Cleanup if needed
    }

private:
    void HandleUpdate(StringHash eventType, VariantMap& eventData)
    {
        float timeStep = eventData[Update::P_TIMESTEP].GetFloat();
        Input* input = GetSubsystem<Input>();

        // Movement speed
        const float MOVE_SPEED = 5.0f;
        const float MOUSE_SENSITIVITY = 0.1f;

        // Mouse look
        if (input->GetMouseMode() == MM_RELATIVE)
        {
            IntVector2 mouseMove = input->GetMouseMove();
            yaw_ += mouseMove.x_ * MOUSE_SENSITIVITY;
            pitch_ += mouseMove.y_ * MOUSE_SENSITIVITY;
            pitch_ = Clamp(pitch_, -90.0f, 90.0f);
            cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));
        }

        // Keyboard movement (WASD)
        if (input->GetKeyDown(KEY_W))
            cameraNode_->Translate(Vector3::FORWARD * MOVE_SPEED * timeStep);
        if (input->GetKeyDown(KEY_S))
            cameraNode_->Translate(Vector3::BACK * MOVE_SPEED * timeStep);
        if (input->GetKeyDown(KEY_A))
            cameraNode_->Translate(Vector3::LEFT * MOVE_SPEED * timeStep);
        if (input->GetKeyDown(KEY_D))
            cameraNode_->Translate(Vector3::RIGHT * MOVE_SPEED * timeStep);
        if (input->GetKeyPress(KEY_SPACE))
            drawDebug_ = !drawDebug_;
    }
    
    void HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData)
    {
        // If draw debug mode is enabled, draw viewport debug geometry. Disable depth test so that we can see the effect of occlusion
        if (drawDebug_)
            GetSubsystem<Renderer>()->DrawDebugGeometry(false);
    }

    void HandleKeyDown(StringHash eventType, VariantMap& eventData)
    {
        int key = eventData[KeyDown::P_KEY].GetInt();
        if (key == KEY_ESCAPE)
            engine_->Exit();
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
        Material* cubeMat = new Material(context_);
        cubeMat->SetTechnique(0, GetSubsystem<ResourceCache>()->GetResource<Technique>(TECHNIQUE_STR)); // FOREST
        cubeMat->SetShaderParameter("MatDiffColor", color);
        cubeMat->SetShadowCullMode(CULL_CW);
        cubeGeom->SetMaterial(cubeMat);
        cubeGeom->SetCastShadows(true);
    }

    SharedPtr<Scene> scene_;
    SharedPtr<Node> cameraNode_;
    DebugHud* debugHud_;
    float yaw_;
    float pitch_;
    bool drawDebug_;
};

URHO3D_DEFINE_APPLICATION_MAIN(MyApp);
