#include "CreatePrimitives.h"
#include "VectorShim.h"

#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/VertexBuffer.h>
#include <Urho3D/Math/Sphere.h>
#include <Urho3D/Math/MathDefs.h>

#include <cstdint>

#ifdef M_PI
#undef M_PI
#endif

using Urho3D::BoundingBox;
using Urho3D::Context;
using Urho3D::Cos;
using Urho3D::Geometry;
using Urho3D::IndexBuffer;
using Urho3D::MASK_NORMAL;
using Urho3D::MASK_POSITION;
using Urho3D::Model;
using Urho3D::M_PI;
using Urho3D::SharedPtr;
using Urho3D::Sin;
using Urho3D::Sphere;
using Urho3D::TRIANGLE_LIST;
using Urho3D::Vector3;
using Urho3D::VertexBuffer;

Urho3D::SharedPtr<Model> CreateSphereModel(Urho3D::Context *context, float radius, int stacks, int slices)
{
    SharedPtr<Model> model(new Model(context));
    SharedPtr<VertexBuffer> vb(new VertexBuffer(context));
    SharedPtr<IndexBuffer> ib(new IndexBuffer(context));
    SharedPtr<Geometry> geom(new Geometry(context));

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
    bb.Define(Sphere(Vector3::ZERO, radius));
    model->SetBoundingBox(bb);

    return model;
}

Urho3D::SharedPtr<Urho3D::Model> CreateCapsuleModel(Urho3D::Context *context, float radius, float height, int rings, int segments)
{
    SharedPtr<Model> model(new Model(context));
    SharedPtr<VertexBuffer> vb(new VertexBuffer(context));
    SharedPtr<IndexBuffer> ib(new IndexBuffer(context));
    SharedPtr<Geometry> geom(new Geometry(context));

    // Enable CPU-side data for physics
    vb->SetShadowed(true);
    ib->SetShadowed(true);

    // Calculate vertices and normals
    ea::vector<Vector3> vertices;
    ea::vector<Vector3> normals;
    const float halfHeight = height * 0.5f;

    // Bottom hemisphere
    for (int ring = 0; ring <= rings; ++ring)
    {
        const float theta = 90.0f * (float(ring) / rings);
        const float y = -radius * Cos(theta) - halfHeight;
        const float r = radius * Sin(theta);

        for (int seg = 0; seg <= segments; ++seg)
        {
            const float phi = 360.0f * float(seg) / segments;
            const float x = r * Cos(phi);
            const float z = r * Sin(phi);
            Vector3 pos(x, y, z);
            vertices.push_back(pos);
            normals.push_back(Vector3(x, y + halfHeight, z).Normalized());
        }
    }

    // Cylinder
    for (int i = 0; i <= 1; ++i)
    {
        const float y = (i == 0 ? -halfHeight : halfHeight);
        for (int seg = 0; seg <= segments; ++seg)
        {
            const float phi = 360.0f * float(seg) / segments;
            const float x = radius * Cos(phi);
            const float z = radius * Sin(phi);
            Vector3 pos(x, y, z);
            vertices.push_back(pos);
            normals.push_back(Vector3(x, 0.0f, z).Normalized());
        }
    }

    // Top hemisphere
    for (int ring = 0; ring <= rings; ++ring)
    {
        const float theta = 90.0f * (float(ring) / rings);
        const float y = radius * Cos(theta) + halfHeight;
        const float r = radius * Sin(theta);

        for (int seg = 0; seg <= segments; ++seg)
        {
            const float phi = 360.0f * float(seg) / segments;
            const float x = r * Cos(phi);
            const float z = r * Sin(phi);
            Vector3 pos(x, y, z);
            vertices.push_back(pos);
            normals.push_back(Vector3(x, y - halfHeight, z).Normalized());
        }
    }

    // Vertex buffer (position, normal)
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
    }
#ifdef USING_RBFX
    vb->Update(vertexData.data());
#else // USING_RBFX
    vb->SetData(vertexData.data());
#endif // USING_RBFX

    // Index buffer
    ea::vector<uint16_t> indices;
    const unsigned ringVerts = segments + 1;

    // Bottom hemisphere indices
    for (uint16_t ring = 0; ring < rings; ++ring)
    {
        for (uint16_t seg = 0; seg < segments; ++seg)
        {
            const uint16_t i0 = ring * ringVerts + seg;
            const uint16_t i1 = (ring + 1) * ringVerts + seg;
            const uint16_t i2 = i0 + 1;
            const uint16_t i3 = i1 + 1;

            // Counterclockwise triangles
            indices.push_back(i0);
            indices.push_back(i1);
            indices.push_back(i2);
            indices.push_back(i2);
            indices.push_back(i1);
            indices.push_back(i3);
        }
    }

    // Cylinder indices
    uint16_t base = (rings + 1) * ringVerts;
    for (uint16_t seg = 0; seg < segments; ++seg)
    {
        const uint16_t i0 = base + seg;
        const uint16_t i1 = base + ringVerts + seg;
        const uint16_t i2 = i0 + 1;
        const uint16_t i3 = i1 + 1;

        // Counterclockwise triangles
        indices.push_back(i0);
        indices.push_back(i1);
        indices.push_back(i2);
        indices.push_back(i2);
        indices.push_back(i1);
        indices.push_back(i3);
    }

    // Top hemisphere indices
    base += 2 * ringVerts;
    for (uint16_t ring = 0; ring < rings; ++ring)
    {
        for (uint16_t seg = 0; seg < segments; ++seg)
        {
            const uint16_t i0 = base + ring * ringVerts + seg;
            const uint16_t i1 = base + (ring + 1) * ringVerts + seg;
            const uint16_t i2 = i0 + 1;
            const uint16_t i3 = i1 + 1;

            // Counterclockwise triangles (flipped for correct orientation)
            indices.push_back(i0);
            indices.push_back(i2);
            indices.push_back(i1);
            indices.push_back(i2);
            indices.push_back(i3);
            indices.push_back(i1);
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
    bb.Define(Vector3(-radius, -halfHeight - radius, -radius), Vector3(radius, halfHeight + radius, radius));
    model->SetBoundingBox(bb);

    return model;
}

/*{
    using namespace Ogre;
    ManualObject* capsule = sceneMgr->createManualObject(name);
    capsule->begin("BaseWhiteNoLighting", RenderOperation::OT_TRIANGLE_LIST);

    const float PI = 3.14159265358979323846f;
    float halfHeight = height * 0.5f;

    // ---------- VERTEX PHASE ----------
    std::vector<Vector3> positions;
    std::vector<Vector3> normals;

    // Bottom hemisphere
    for (int ring = 0; ring <= rings; ++ring)
    {
        float theta = (PI / 2.0f) * (float(ring) / rings);  // 0 to PI/2
        float y = -radius * std::cos(theta) - halfHeight;
        float r = radius * std::sin(theta);

        for (int seg = 0; seg <= segments; ++seg)
        {
            float phi = 2.0f * PI * float(seg) / segments;
            float x = r * std::cos(phi);
            float z = r * std::sin(phi);
            Vector3 pos(x, y, z);
            positions.push_back(pos);
            normals.push_back((Vector3(x, y + halfHeight, z)).normalisedCopy());
        }
    }

    // Cylinder
    for (int i = 0; i <= 1; ++i)
    {
        float y = (i == 0 ? -halfHeight : halfHeight);
        for (int seg = 0; seg <= segments; ++seg)
        {
            float phi = 2.0f * PI * float(seg) / segments;
            float x = radius * std::cos(phi);
            float z = radius * std::sin(phi);
            Vector3 pos(x, y, z);
            positions.push_back(pos);
            normals.push_back(Vector3(x, 0, z).normalisedCopy());
        }
    }

    // Top hemisphere
    for (int ring = 0; ring <= rings; ++ring)
    {
        float theta = (PI / 2.0f) * (float(ring) / rings);  // 0 to PI/2
        float y = radius * std::cos(theta) + halfHeight;
        float r = radius * std::sin(theta);

        for (int seg = 0; seg <= segments; ++seg)
        {
            float phi = 2.0f * PI * float(seg) / segments;
            float x = r * std::cos(phi);
            float z = r * std::sin(phi);
            Vector3 pos(x, y, z);
            positions.push_back(pos);
            normals.push_back((Vector3(x, y - halfHeight, z)).normalisedCopy());
        }
    }

    // Add vertices
    for (size_t i = 0; i < positions.size(); ++i)
    {
        capsule->position(positions[i]);
        capsule->normal(normals[i]);
    }

    // ---------- INDEX PHASE ----------
    int ringVerts = segments + 1;

    // Bottom hemisphere indices
    for (int ring = 0; ring < rings; ++ring)
    {
        for (int seg = 0; seg < segments; ++seg)
        {
            int i0 = ring * ringVerts + seg;
            int i1 = (ring + 1) * ringVerts + seg;
            int i2 = i0 + 1;
            int i3 = i1 + 1;

            capsule->triangle(i0, i1, i2);
            capsule->triangle(i2, i1, i3);
        }
    }

    int base = (rings + 1) * ringVerts;
    // Cylinder indices
    for (int seg = 0; seg < segments; ++seg)
    {
        int i0 = base + seg;
        int i1 = base + ringVerts + seg;
        int i2 = i0 + 1;
        int i3 = i1 + 1;

        capsule->triangle(i0, i1, i2);
        capsule->triangle(i2, i1, i3);
    }

    base += 2 * ringVerts;
    // Top hemisphere indices
    for (int ring = 0; ring < rings; ++ring)
    {
        for (int seg = 0; seg < segments; ++seg)
        {
            int i0 = base + ring * ringVerts + seg;
            int i1 = base + (ring + 1) * ringVerts + seg;
            int i2 = i0 + 1;
            int i3 = i1 + 1;

            capsule->triangle(i0, i2, i1);
            capsule->triangle(i2, i3, i1);
        }
    }

    capsule->end();
    capsule->convertToMesh(name);
}*/
