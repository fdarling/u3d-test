#ifndef SCENELOADER_H
#define SCENELOADER_H

#include <string>

namespace Urho3D {

class Scene;
class Context;

} // namespace Urho3D

void loadSceneWithAssimp(const std::string& filename, Urho3D::Node* sceneMgr, Urho3D::Context* context);

#endif // SCENELOADER_H
