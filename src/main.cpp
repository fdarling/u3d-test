#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Engine/Application.h>
#include <Urho3D/Engine/EngineDefs.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/GraphicsEvents.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/Viewport.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/Input/InputEvents.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#ifdef USING_RBFX
#include <Urho3D/RenderPipeline/RenderPipeline.h>
#endif // USING_RBFX
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Resource/ResourceCache.h>
#ifdef USING_RBFX
#include <Urho3D/SystemUI/DebugHud.h>
#else // USING_RBFX
#include <Urho3D/Engine/DebugHud.h>
#endif // USING_RBFX
#include <Urho3D/IO/Log.h>

// for debug UI
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>
#include <Urho3D/UI/UIEvents.h>
#include <Urho3D/UI/Window.h>

#include "VectorShim.h"
#include "SceneLoader.h"
#include "Player.h"
#include "JumpPad.h"
#include "Ladder.h"
#include "Ball.h"
#include "globals.h"

#include <Urho3D/ThirdParty/Bullet/BulletDynamics/Dynamics/btRigidBody.h>

using namespace Urho3D;

class MyApp : public Application
{
    URHO3D_OBJECT(MyApp, Application)

public:
    MyApp(Context *context) :
        Application(context),
        yaw_(0.0f),
        pitch_(0.0f),
        cameraMode_(CameraMode::ThirdPerson),
        drawDebug_(false),
        drawPhysicsDebug_(true),
        shadowsEnabled_(true),
        ssaoEnabled_(true)
    {
    }

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
        ResourceCache * const cache = GetSubsystem<ResourceCache>();
#ifdef DEBUGGING_LADDER
        UI * const ui = GetSubsystem<UI>();
        {
            UIElement * const root = ui->GetRoot();
            root->SetDefaultStyle(cache->GetResource<XMLFile>("UI/DefaultStyle.xml"));

            Window * const w = new Window(context_);
            w->SetStyleAuto();
            w->SetLayoutMode(LayoutMode::LM_VERTICAL);
            root->AddChild(w);
            w->SetPosition(IntVector2(100, 50));

            debugOnLadderText_ = new Text(context_);
            w->AddChild(debugOnLadderText_);
            debugOnLadderText_->SetStyleAuto();

            debugOnGroundText_ = new Text(context_);
            w->AddChild(debugOnGroundText_);
            debugOnGroundText_->SetStyleAuto();

            debugLadderCaseText_ = new Text(context_);
            w->AddChild(debugLadderCaseText_);
            debugLadderCaseText_->SetStyleAuto();

            debugNormalPitchText_ = new Text(context_);
            w->AddChild(debugNormalPitchText_);
            debugNormalPitchText_->SetStyleAuto();

            debugDotProductText_ = new Text(context_);
            w->AddChild(debugDotProductText_);
            debugDotProductText_->SetStyleAuto();

            debugCameraDirText_ = new Text(context_);
            w->AddChild(debugCameraDirText_);
            debugCameraDirText_->SetStyleAuto();

            debugRawWalkingDirText_ = new Text(context_);
            w->AddChild(debugRawWalkingDirText_);
            debugRawWalkingDirText_->SetStyleAuto();

            debugWalkingDirText_ = new Text(context_);
            w->AddChild(debugWalkingDirText_);
            debugWalkingDirText_->SetStyleAuto();

            debugLadderNormalText_ = new Text(context_);
            w->AddChild(debugLadderNormalText_);
            debugLadderNormalText_->SetStyleAuto();
        }
#endif // DEBUGGING_LADDER

        // Create scene
        scene_ = new Scene(context_);
        octree_ = scene_->CreateComponent<Octree>();
        physicsWorld_ = scene_->CreateComponent<PhysicsWorld>();
        // physicsWorld_->SetMaxSubSteps(10); // default is 0 for unlimited
        // physicsWorld_->SetFps(240); // default is 60
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
            // settings.renderBufferManager_.colorSpace_ = RenderPipelineColorSpace::GammaLDR; // default
            // settings.renderBufferManager_.colorSpace_ = RenderPipelineColorSpace::LinearLDR;
            // settings.renderBufferManager_.colorSpace_ = RenderPipelineColorSpace::LinearHDR;
            // settings.renderBufferManager_.colorSpace_ = RenderPipelineColorSpace::Optimized;
            // settings.sceneProcessor_.pcfKernelSize_ = 1; // default
            settings.sceneProcessor_.pcfKernelSize_ = 2; // default
            // settings.sceneProcessor_.normalOffsetScale_ = 1.0; // default
            // settings.sceneProcessor_.directionalShadowSize_ = 1024; // default
            // settings.sceneProcessor_.spotShadowSize_ = 1024; // default
            // settings.sceneProcessor_.pointShadowSize_ = 256; // default
            settings.sceneProcessor_.pointShadowSize_ = 1024;
            // settings.sceneProcessor_.ambientMode_ = DrawableAmbientMode::Directional; // default
            // settings.sceneProcessor_.ambientMode_ = DrawableAmbientMode::Constant;
            // settings.sceneProcessor_.ambientMode_ = DrawableAmbientMode::Flat;
            // settings.shadowMapAllocator_.enableVarianceShadowMaps_ = false; // default
            // settings.shadowMapAllocator_.enableVarianceShadowMaps_ = true;
            // settings.shadowMapAllocator_.varianceShadowMapMultiSample_ = 1; // default
            // settings.shadowMapAllocator_.varianceShadowMapMultiSample_ = 4;
            // settings.shadowMapAllocator_.use16bitShadowMaps_ = false; // default
            // settings.shadowMapAllocator_.shadowAtlasPageSize_ = 2048; // default
            settings.shadowMapAllocator_.shadowAtlasPageSize_ = 8192; // trying
            // settings.shadowMapAllocator_.depthBiasScale_ = 1.0; // default
            // settings.shadowMapAllocator_.depthBiasScale_ = 0.5;
            // settings.shadowMapAllocator_.depthBiasOffset_ = 0.0; // default
            // settings.shadowMapAllocator_.depthBiasOffset_ = 0.0001;
            renderPipeline->SetSettings(settings);
            renderPipeline->SetRenderPassEnabled(eastl::string("Postprocess: SSAO"), ssaoEnabled_);
        }
#endif // USING_RBFX

        loadSceneWithAssimp("../assets/test_scene_torus.glb", scene_, context_);

        // TODO store pointers, we are leaking these object currently!
        player_ = new Player(scene_, Vector3(6, PLAYER_HEIGHT/2.0+0.01, 0));
        JumpPad * const jumpPad = new JumpPad(scene_, Vector3(2.0, 0.25, 0.0), Vector3(2.0, 0.5, 2.0));
        Ladder * const ladder = new Ladder(scene_, Vector3(4.0, 8.0, 4.0), Vector3(2.0, 16.0, 2.0));

        // stick a ball to the ladder
        {
            Ball * const redBall = new Ball(scene_, Vector3(4.0, 8.0, 4.0) + Vector3(1.25, 0.0, 0.0), Vector3::ZERO, Color(1.0, 0.0, 0.0));
            redBall->GetNode()->GetComponent<RigidBody>()->SetUseGravity(false);
            ladder->ConstrainNode(redBall->GetNode());

            // test removing the constraint
            // ladder->UnconstrainNode(redBall->GetNode());
            // redBall->GetNode()->GetComponent<RigidBody>()->SetUseGravity(true);
        }

        // Camera
        cameraNode_ = scene_->CreateChild("Camera");
        cameraPos_ = Vector3(0.0f, 5.0f, -20.0f);
        // if (0)
        {
            // for looking at the torus
            cameraPos_ = Vector3(1.15f, 2.97f, -7.82f);
            pitch_ = 28.2;
            yaw_ = -17.8;
        }
        camera_ = cameraNode_->CreateComponent<Camera>();
        camera_->SetFarClip(300.0f);
        UpdateCamera();

        // Viewport
        Renderer * const renderer = GetSubsystem<Renderer>();
        SharedPtr<Viewport> viewport(new Viewport(context_, scene_, camera_));
        renderer->SetViewport(0, viewport);

        // Debug HUD for FPS
        debugHud_ = engine_->CreateDebugHud();
#ifndef USING_RBFX
        debugHud_->SetDefaultStyle(cache->GetResource<XMLFile>("UI/DefaultStyle.xml"));
        debugHud_->SetMode(DEBUGHUD_SHOW_ALL);
        // Font * const font = cache->GetResource<Font>("Fonts/Anonymous Pro.ttf");
#endif // USING_RBFX

        // Set mouse mode for FPS control
        Input * const input = GetSubsystem<Input>();
        // input->SetMouseVisible(false);
        input->SetMouseMode(MM_RELATIVE);

        // Subscribe to events
        SubscribeToEvent(E_KEYDOWN, URHO3D_HANDLER(MyApp, HandleKeyDown));
        SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(MyApp, HandleUpdate));
        SubscribeToEvent(E_MOUSEMOVE, URHO3D_HANDLER(MyApp, HandleMouseMove));
        SubscribeToEvent(E_POSTRENDERUPDATE, URHO3D_HANDLER(MyApp, HandlePostRenderUpdate));
    }

    virtual void Stop() override {}

    void HandleKeyDown(StringHash eventType, VariantMap &eventData)
    {
        int key = eventData[KeyDown::P_KEY].GetInt();
        if (key == KEY_ESCAPE) {
            engine_->Exit();
        }
    }

    void HandleUpdate(StringHash eventType, VariantMap &eventData)
    {
        static const float WALK_SPEED = 10.0f;
        const float timeStep = eventData[Update::P_TIMESTEP].GetFloat();
        const float walkDistance = WALK_SPEED * timeStep;

        // get input object for testing keyboard/mouse presses
        Input * const input = GetSubsystem<Input>();

        // WASD movement keys
        Vector3 wasdDir(Vector3::ZERO);
        if ( input->GetKeyDown(KEY_W) && !input->GetKeyDown(KEY_S))
            wasdDir += Vector3::FORWARD;
        if (!input->GetKeyDown(KEY_W) &&  input->GetKeyDown(KEY_S))
            wasdDir += Vector3::BACK;
        if ( input->GetKeyDown(KEY_A) && !input->GetKeyDown(KEY_D))
            wasdDir += Vector3::LEFT;
        if (!input->GetKeyDown(KEY_A) &&  input->GetKeyDown(KEY_D))
            wasdDir += Vector3::RIGHT;
        if (cameraMode_ == CameraMode::FreeLook)
        {
            if ( input->GetKeyDown(KEY_LCTRL) && !input->GetKeyDown(KEY_SPACE))
                wasdDir += Vector3::DOWN;
            if (!input->GetKeyDown(KEY_LCTRL) &&  input->GetKeyDown(KEY_SPACE))
                wasdDir += Vector3::UP;
        }

        // IJKL movement keys
        Vector3 ijklDir(Vector3::ZERO);
        if ( input->GetKeyDown(KEY_I) && !input->GetKeyDown(KEY_K))
            ijklDir += Vector3::FORWARD;
        if (!input->GetKeyDown(KEY_I) &&  input->GetKeyDown(KEY_K))
            ijklDir += Vector3::BACK;
        if ( input->GetKeyDown(KEY_J) && !input->GetKeyDown(KEY_L))
            ijklDir += Vector3::LEFT;
        if (!input->GetKeyDown(KEY_J) &&  input->GetKeyDown(KEY_L))
            ijklDir += Vector3::RIGHT;

        auto adjustWalkDir = [&] (Player *player, const Vector3 &faceDir, const Vector3 &walkDir) -> Vector3 {
#ifdef DEBUGGING_LADDER
            this->debugOnLadderText_->SetText(eastl::string("On Ladder?: ") + (player->IsOnLadder() ? "true" : "false"));
            this->debugOnGroundText_->SetText(eastl::string("On Ground?: ") + (player->IsOnGround() ? "true" : "false"));
#endif // DEBUGGING_LADDER

            // leave pitch unmodified if we aren't even on a ladder
            if (!player->IsOnLadder())
            {
#ifdef DEBUGGING_LADDER
                this->debugLadderCaseText_->SetText(eastl::string("Ladder Case: not on ladder!"));
#endif // DEBUGGING_LADDER
                return walkDir;
            }

            // if we are on the ground and trying to leave the ladder, walk normally
            const bool walkingTowardsLadder = player->IsFacingLadder(walkDir);
            if (player->IsOnGround() && !walkingTowardsLadder)
            {
#ifdef DEBUGGING_LADDER
                this->debugLadderCaseText_->SetText(eastl::string("Ladder Case: on ground / walking away"));
#endif // DEBUGGING_LADDER
                return walkDir;
            }

            // check to see if we are at the top of the ladder (maximum altitude)
            const bool aboveLadderVertically = player->IsAboveLadderVertically();
            if (aboveLadderVertically)
            {
                // are we on top of the ladder (in the sense of it being a platform)?
                const bool aboveLadderHorizontally = player->IsAboveLadderHorizontally();
                if (aboveLadderHorizontally)
                {
#ifdef DEBUGGING_LADDER
                    this->debugLadderCaseText_->SetText(eastl::string("Ladder Case: already on top"));
#endif // DEBUGGING_LADDER
                    return walkDir;
                }

                // are we walking onto the top of the ladder?
                if (walkingTowardsLadder)
                {
#ifdef DEBUGGING_LADDER
                    this->debugLadderCaseText_->SetText(eastl::string("Ladder Case: walking onto top"));
#endif // DEBUGGING_LADDER
                    return walkDir;
                }
            }

            // we are not at the top of the ladder, or we are at the top but
            // trying to climb down not up
            const Vector3 ladderNormal = player->GetLadderNormal();
            const Vector3 rotAxis = ladderNormal.CrossProduct(Vector3::UP);
            const float normalDot = ladderNormal.DotProduct(walkDir);
            const Vector3 normalComponent = (ladderNormal*normalDot);
            if (normalComponent == Vector3::ZERO)
            {
#ifdef DEBUGGING_LADDER
                this->debugLadderCaseText_->SetText(eastl::string("Ladder Case: zero normal component"));
#endif // DEBUGGING_LADDER
                return walkDir;
            }
            const Vector3 verticalComponent = Vector3::UP*(walkDir.DotProduct(Vector3::UP));
            const float normalPitch = (verticalComponent + normalComponent).Angle(-ladderNormal);
            const float targetPitch = Clamp(2.0f*(normalPitch - 45.0f), -89.9f, 89.9f);
            const Vector3 newWalkDir = Quaternion(targetPitch - normalPitch, rotAxis).RotationMatrix()*walkDir;

#ifdef DEBUGGING_LADDER
            this->debugLadderCaseText_->SetText(eastl::string("Ladder Case: modifying"));
            this->debugNormalPitchText_->SetText(eastl::string("Normal Pitch: ") + eastl::to_string(normalPitch));
            this->debugDotProductText_->SetText(eastl::string("Dot Product: ") + eastl::to_string(normalDot));
            this->debugCameraDirText_->SetText(eastl::string("Camera Dir: ") + faceDir.ToString());
            this->debugRawWalkingDirText_->SetText(eastl::string("Original Walking Dir: ") + walkDir.ToString());
            this->debugWalkingDirText_->SetText(eastl::string("Modified Walking Dir: ") + newWalkDir.ToString());
            this->debugLadderNormalText_->SetText(eastl::string("Ladder Normal: ") + ladderNormal.ToString());
#endif // DEBUGGING_LADDER

            return newWalkDir;
        };

        // determine how to use the keys
        const bool usingWasdForWalking = (cameraMode_ != CameraMode::FreeLook);
        const Matrix3 horizRotMat = Quaternion(yaw_, Vector3::UP).RotationMatrix();
        const Matrix3 fullRotMat = Quaternion(pitch_, yaw_, 0.0f).RotationMatrix();
        const Vector3 originalWalkDir = (usingWasdForWalking ? wasdDir : ijklDir).Normalized();
        const Vector3 walkDir = player_->IsOnLadder() ? adjustWalkDir(player_, cameraNode_->GetWorldDirection().Normalized(), fullRotMat*originalWalkDir) : horizRotMat*originalWalkDir;
        const bool wantsJump = input->GetKeyDown(usingWasdForWalking ? KEY_SPACE : KEY_RSHIFT);

        if (cameraMode_ == CameraMode::FreeLook)
        {
            // move camera
            wasdDir = fullRotMat*wasdDir;
            if (wasdDir != Vector3::ZERO)
            {
                cameraPos_ += wasdDir.Normalized()*walkDistance;
                // std::cout << "camera pos: (" << cameraPos_.x_ << "," << cameraPos_.y_ << "," << cameraPos_.z_ << ")" << std::endl;
            }
        }
        player_->SetWalkDirection(walkDir);
        player_->SetJumping(wantsJump);

        // cycle camera mode
        if (input->GetKeyPress(KEY_T))
            cameraMode_ = static_cast<CameraMode>((static_cast<int>(cameraMode_)+1)%static_cast<int>(CameraMode::MAX));
        UpdateCamera();

        // toggle graphics debug rendering
        if (input->GetKeyPress(KEY_Z))
            drawDebug_ = !drawDebug_;

        // toggle wireframe rendering
        if (input->GetKeyPress(KEY_X))
            camera_->SetFillMode(camera_->GetFillMode() == FILL_WIREFRAME ? FILL_SOLID : FILL_WIREFRAME);

        // toggle debug drawing
        if (input->GetKeyPress(KEY_C))
            drawPhysicsDebug_ = !drawPhysicsDebug_;

        // toggle shadows
        if (input->GetKeyPress(KEY_M))
        {
            shadowsEnabled_ = !shadowsEnabled_;
            ea::vector<Light*> lights;
#ifdef USING_RBFX
            scene_->FindComponents<Light>(lights, ComponentSearchFlag::SelfOrChildrenRecursive);
#else
            scene_->GetComponents<Light>(lights, true);
#endif // USING_RBFX
            for (Light * const light : lights)
                light->SetCastShadows(shadowsEnabled_);
        }

#ifdef USING_RBFX
        // toggle SSAO
        if (input->GetKeyPress(KEY_O))
        {
            ssaoEnabled_ = !ssaoEnabled_;
            RenderPipeline * const renderPipeline = scene_->GetComponent<RenderPipeline>();
            renderPipeline->SetRenderPassEnabled(eastl::string("Postprocess: SSAO"), ssaoEnabled_);
        }
#endif // USING_RBFX

        // toggle mouse grabbing / mouselook
        if (input->GetKeyPress(KEY_TAB))
        {
            const bool wasRelative = (input->GetMouseMode() == MM_RELATIVE);
            input->SetMouseMode(wasRelative ? MM_ABSOLUTE : MM_RELATIVE);
            input->SetMouseVisible(wasRelative);
        }

        // player state advancement
        player_->Advance();

        // shoot sphere on left mouse click
        if (input->GetMouseButtonPress(MOUSEB_LEFT))
        {
            static const float BALL_SPEED = 25.0;
            Ball * const ball = new Ball(scene_, cameraNode_->GetWorldPosition(), cameraNode_->GetWorldDirection().Normalized()*BALL_SPEED, Color(1.0f, 1.0f, 1.0f));
        }

        // Update debug HUD (shows FPS)
        debugHud_->SetMode(DEBUGHUD_SHOW_ALL);
    }

    void HandlePostRenderUpdate(StringHash eventType, VariantMap &eventData)
    {
        if (drawDebug_)
            GetSubsystem<Renderer>()->DrawDebugGeometry(false);
        if (drawPhysicsDebug_)
            physicsWorld_->DrawDebugGeometry(true);
    }

    void HandleMouseMove(StringHash eventType, VariantMap &eventData)
    {
        Input * const input = GetSubsystem<Input>();
        if (input->GetMouseMode() != MM_RELATIVE) return;

        const int dx = eventData[MouseMove::P_DX].GetInt();
        const int dy = eventData[MouseMove::P_DY].GetInt();

        const float mouseSensitivity = 0.2f;
        yaw_ += dx * mouseSensitivity;
        pitch_ += dy * mouseSensitivity;
        pitch_ = Clamp(pitch_, -90.0f, 90.0f);

        // std::cout << "camera pitch/yaw: " << pitch_ << "," << yaw_ << std::endl;
        // UpdateCamera();
    }
protected:
    void UpdateCamera()
    {
        const Quaternion quat = Quaternion(pitch_, yaw_, 0.0f);
        Vector3 cPos;
        if (cameraMode_ == CameraMode::FreeLook)
            cPos = cameraPos_;
        else
        {
            cPos = player_->GetNode()->GetPosition();
            if (cameraMode_ == CameraMode::ThirdPerson)
                cPos += quat.RotationMatrix()*Vector3(0.0, 2.0, -10.0);
        }
        cameraNode_->SetPosition(cPos);
        cameraNode_->SetRotation(quat);
    }
    enum class CameraMode
    {
        FreeLook,
        FirstPerson,
        ThirdPerson,
        MAX
    };
    SharedPtr<Scene> scene_;
    SharedPtr<Node> cameraNode_;
    SharedPtr<DebugHud> debugHud_;
    SharedPtr<PhysicsWorld> physicsWorld_;
    SharedPtr<Octree> octree_;
    SharedPtr<Zone> zone_;
    SharedPtr<Camera> camera_;
    SharedPtr<Player> player_;
#ifdef DEBUGGING_LADDER
    SharedPtr<Text> debugOnLadderText_;
    SharedPtr<Text> debugOnGroundText_;
    SharedPtr<Text> debugLadderCaseText_;
    SharedPtr<Text> debugNormalPitchText_;
    SharedPtr<Text> debugDotProductText_;
    SharedPtr<Text> debugCameraDirText_;
    SharedPtr<Text> debugRawWalkingDirText_;
    SharedPtr<Text> debugWalkingDirText_;
    SharedPtr<Text> debugLadderNormalText_;
#endif // DEBUGGING_LADDER
    Vector3 cameraPos_;
    float yaw_;
    float pitch_;
    CameraMode cameraMode_;
    bool drawDebug_;
    bool drawPhysicsDebug_;
    bool shadowsEnabled_;
    bool ssaoEnabled_;
};

URHO3D_DEFINE_APPLICATION_MAIN(MyApp);
