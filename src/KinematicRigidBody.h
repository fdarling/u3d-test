#pragma once

#include <Urho3D/Physics/RigidBody.h>

#include <Urho3D/ThirdParty/Bullet/LinearMath/btTransform.h>

class KinematicRigidBody : public Urho3D::RigidBody
{
    // URHO3D_OBJECT(KinematicRigidBody, Urho3D::Component);
public:
    explicit KinematicRigidBody(Urho3D::Context *context);
    ~KinematicRigidBody() override;

    void getWorldTransform(btTransform &worldTrans) const override;
    void setWorldTransform(const btTransform &worldTrans) override;
public:
    void setOverrideTransform(const btTransform &worldTrans);
    const btTransform & getOverrideTransform() const {return overrideTrans_;}
    btTransform overrideTrans_;
    bool overriding_;
};
