# Dependencies

## U3D (Urho3D fork)

```
git clone https://github.com/u3d-community/U3D.git
cd U3D
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/apps/U3D
cmake --build . --config Release --target install
```

# Compiling

```
mkdir build
cd build
cmake .. -DCMAKE_MODULE_PATH=~/apps/U3D/share/Urho3D/cmake/Modules -DURHO3D_HOME=~/apps/U3D
cmake --build .
```

# Running 

```
URHO3D_PREFIX_PATH=~/apps/U3D/share/Urho3D/resources bin/u3d-test
```

# Other Resources:

* [Urho3D](https://urho3d.io/) (project died, supposedly due to hostile takeover), GitHub archived repo: https://github.com/urho3d/urho3d
* [U3D](https://u3d.io/), GitHub repo: https://github.com/u3d-community/U3D
* [rbfx](https://rebelfork.io/) aka "Rebel Fork Framework", GitHub repo: https://github.com/rbfx/rbfx
* [Dry](https://dry.luckey.games/) aka "Dry Engine", GitLab repo: https://gitlab.com/luckeyproductions/dry
