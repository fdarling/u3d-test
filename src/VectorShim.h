#pragma once


#ifndef USING_RBFX

#include <Urho3D/Container/Vector.h>

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

#endif // USING_RBFX
