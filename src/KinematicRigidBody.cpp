#include "KinematicRigidBody.h"

KinematicRigidBody::KinematicRigidBody(Urho3D::Context *context) : RigidBody(context), overriding_(false)
{
    overrideTrans_.setIdentity();
}

KinematicRigidBody::~KinematicRigidBody() = default;

void KinematicRigidBody::getWorldTransform(btTransform &worldTrans) const
{
    RigidBody::getWorldTransform(worldTrans);
    if (overriding_)
        worldTrans = overrideTrans_;
}

void KinematicRigidBody::setWorldTransform(const btTransform &worldTrans)
{
    RigidBody::setWorldTransform(worldTrans);
}

void KinematicRigidBody::setOverrideTransform(const btTransform &worldTrans)
{
    overrideTrans_ = worldTrans;
    overriding_ = true;
}
