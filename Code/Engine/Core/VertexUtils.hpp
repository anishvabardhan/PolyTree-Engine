#pragma once

#include "Engine/Core/Rgba8.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/AABB2.hpp"
#include "Engine/Math/AABB3.hpp"
#include "Engine/Math/Plane3D.hpp"
#include "Engine/Renderer/Camera.hpp"

#include <math.h>
#include <vector>
#include <string>

struct Vec3;
struct OBB2;
struct OBB3;
struct Mat44;
struct Vertex_PCU;
struct Vertex_PCUTBN;
struct MeshVertex_PCU;
struct MeshVertex_PCUTBN;

void TransformVertexArrayXY3D(int numVerts, Vertex_PCU* verts, float uniformScaleXY, float rotationDegreesAboutZ, Vec2 const& translationXY);
void TransformVertexArray3D(std::vector<Vertex_PCU>& verts, Mat44 const& transform);
void TransformVertexArray3D(std::vector<Vertex_PCUTBN>& verts, Mat44 const& transform, bool willTransformNormals = false);
void TransformVertexArray3D(std::vector<MeshVertex_PCU>& verts, Mat44 const& transform, bool willTransformNormals = false);

void AddVertsForDirectedSector2D(std::vector<Vertex_PCU>& vertices, Vec2 const& sectorTip, Vec2 const& sectorForwardNormal, float sectorApertureDegrees, float sectorRadius, Rgba8 const& color);
void AddVertsForOrientedSector2D(std::vector<Vertex_PCU>& vertices, Vec2 const& sectorTip, float sectorForwardDegrees, float sectorApertureDegrees, float sectorRadius, Rgba8 const& color);
void AddVertsForCapsule2D(std::vector<Vertex_PCU>& vertices, Vec2 const& boneStart, Vec2 const& boneEnd, float radius, Rgba8 const& color);
void AddVertsForDisc2D(std::vector<Vertex_PCU>& vertices, Vec2 const& center, float radius, Rgba8 const& color);
void AddVertsForAABB2D(std::vector<Vertex_PCU>& vertices, AABB2 const& bounds, Rgba8 const& color, Vec2 uvMins = Vec2::ZERO, Vec2 uvMaxs = Vec2::ONE);
void AddVertsForOBB2D(std::vector<Vertex_PCU>& vertices, OBB2 const& box, Rgba8 const& color);
void AddVertsForOBB3D(std::vector<Vertex_PCU>& vertices, OBB3 const& box, Rgba8 const& color, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForLineSegment2D(std::vector<Vertex_PCU>& vertices, Vec2 const& start, Vec2 const& end, float thickness, Rgba8 const& color);
void AddVertsForArrow2D(std::vector<Vertex_PCU>& vertices, Vec2 tailPos, Vec2 tipPos, float arrowSize, float lineThickness, Rgba8 const& color);
void AddVertsForQuad3D(std::vector<Vertex_PCU>& vertices, Vec3 const& bottomLeft, Vec3 const& bottomRight, Vec3 const& topRight, Vec3 const& topLeft, Rgba8 const& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForQuad3D(std::vector<Vertex_PCU>& vertices, Vec3 const& bottomLeft, Vec3 const& bottomRight, Vec3 const& topRight, Vec3 const& topLeft, Rgba8 const& startColor = Rgba8::WHITE, Rgba8 const& endColor = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForQuad3D(std::vector<Vertex_PCUTBN>& vertices, Vec3 const& bottomLeft, Vec3 const& bottomRight, Vec3 const& topRight, Vec3 const& topLeft, Rgba8 const& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForQuad3D(std::vector<Vertex_PCU>& vertices, std::vector<unsigned int>& indices, int indexOffset, Vec3 const& bottomLeft, Vec3 const& bottomRight, Vec3 const& topRight, Vec3 const& topLeft, Rgba8 const& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForQuad3D(std::vector<Vertex_PCUTBN>& vertices, std::vector<unsigned int>& indices, int indexOffset, Vec3 const& bottomLeft, Vec3 const& bottomRight, Vec3 const& topRight, Vec3 const& topLeft, Rgba8 const& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForAABB3D(std::vector<Vertex_PCU>& vertices, AABB3 const& bounds, Rgba8 const& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForAABB3D(std::vector<Vertex_PCU>& vertices, std::vector<unsigned int>& indices, int indexOffset, AABB3 const& bounds, Rgba8 const& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForAABB3D(std::vector<Vertex_PCUTBN>& vertices, AABB3 const& bounds, Rgba8 const& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForAABB3D(std::vector<Vertex_PCUTBN>& vertices, std::vector<unsigned int>& indices, int indexOffset, AABB3 const& bounds, Rgba8 const& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForPlane3D(std::vector<Vertex_PCU>& vertices, Plane3D const& plane);
void AddVertsForWireAABB3D(std::vector<Vertex_PCU>& vertices, AABB3 const& bounds, Rgba8 const& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForSphere3D(std::vector<Vertex_PCU>& vertices, Vec3 const& center, float radius, Rgba8 const& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE, int numLatitudeSlices = 8, int numLongitudeSlices = 8);
void AddVertsForSphere3D(std::vector<Vertex_PCUTBN>& vertices, Vec3 const& center, float radius, Rgba8 const& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE, int numLatitudeSlices = 8, int numLongitudeSlices = 8);
void AddVertsForZSphere(std::vector<Vertex_PCUTBN>& verts, std::vector<unsigned int>& indexes, const Vec3& center, float radius, const Rgba8& color, const AABB2& UVs, int numLatitudeSlices, int numLongitudeSlices);
void AddVertsForWireZSphere3D(std::vector<Vertex_PCU>& vertices, Vec3 const& center, float radius, Rgba8 const& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE, int numLatitudeSlices = 8, int numLongitudeSlices = 8);
void AddVertsForCylinder3D(std::vector<Vertex_PCU>& vertices, Vec3 const& start, Vec3 const& end, float radius, Rgba8 const& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE, int numSlices = 8);
void AddVertsForCylinder3D(std::vector<Vertex_PCU>& vertices, Vec3 const& start, Vec3 const& end, float radius, Rgba8 const& startColor = Rgba8::WHITE, Rgba8 const& endColor = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE, int numSlices = 8);
void AddVertsForZCylinder3D(std::vector<Vertex_PCU>& vertices, Vec3 const& start, float const& height, float radius, Rgba8 const& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE, int numSlices = 8);
void AddVertsForZWireCylinder3D(std::vector<Vertex_PCU>& vertices, Vec3 const& start, float const& height, float radius, Rgba8 const& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE, int numSlices = 8);
void AddVertsForCone3D(std::vector<Vertex_PCU>& vertices, Vec3 const& start, Vec3 const& end, float radius, Rgba8 const& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE, int numSlices = 8);
void AddVertsForArrow(std::vector<Vertex_PCU>& vertices, Vec3 const& start, Vec3 const& end, float radius, Rgba8 const& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForHollowHexagon3D(std::vector<Vertex_PCU>& vertices, Vec3 const& center, float width, float thickness, Rgba8 const& color);
void AddVertsForHexagon3D(std::vector<Vertex_PCU>& vertices, Vec3 const& center, float width, float thickness, Rgba8 const& color);

void CalculateTangentSpaceBasisVectors(std::vector<Vertex_PCUTBN>& vertices, std::vector<unsigned int> indices, bool computeNormals, bool computeTangents);
void CalculateTangentSpaceBasisVectors(std::vector<MeshVertex_PCUTBN>& vertices, std::vector<unsigned int> indices, bool computeNormals, bool computeTangents);

AABB2 GetVertexBounds2D(std::vector<Vertex_PCU> const& verts);