#include "Engine/Core/VertexUtils.hpp"

#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/AABB2.hpp"
#include "Engine/Math/OBB2.hpp"
#include "Engine/Math/OBB3.hpp"
#include "Engine/Math/MathUtils.hpp"

#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Core/Vertex_PCUTBN.hpp"
#include "Engine/Core/MeshVertex_PCU.hpp"

void TransformVertexArrayXY3D(int numVerts, Vertex_PCU* verts, float uniformScaleXY, float rotationDegreesAboutZ, Vec2 const& translationXY)
{
	for (int index = 0; index < numVerts; index++)
	{
		Vec3& position = verts[index].m_position;

		float thetaRadians = static_cast<float>(atan2(position.y, position.x));
		float distanceXY = sqrtf((position.x * position.x) + (position.y * position.y));

		float rotateRadians = rotationDegreesAboutZ * (3.14f / 180.0f);
		thetaRadians += rotateRadians;

		distanceXY *= uniformScaleXY;

		position.x = distanceXY * cosf(thetaRadians);
		position.y = distanceXY * sinf(thetaRadians);

		position.x += translationXY.x;
		position.y += translationXY.y;
	}
}

void TransformVertexArray3D(std::vector<Vertex_PCU>& verts, Mat44 const& transform)
{
	for (size_t index = 0; index < verts.size(); index++)
	{
		verts[index].m_position = transform.TransformPosition3D(verts[index].m_position);
	}
}

void TransformVertexArray3D(std::vector<Vertex_PCUTBN>& verts, Mat44 const& transform, bool willTransformNormals)
{
	for (size_t index = 0; index < verts.size(); index++)
	{
		verts[index].m_position = transform.TransformPosition3D(verts[index].m_position);

		if (willTransformNormals)
		{
			verts[index].m_normal = transform.TransformVectorQuantity3D(verts[index].m_normal);
		}
	}
}

void TransformVertexArray3D(std::vector<MeshVertex_PCU>& verts, Mat44 const& transform, bool willTransformNormals)
{
	UNUSED(willTransformNormals);

	for (size_t index = 0; index < verts.size(); index++)
	{
		verts[index].m_position = transform.TransformPosition3D(verts[index].m_position);
	}
}

void AddVertsForDirectedSector2D(std::vector<Vertex_PCU>& vertices, Vec2 const& sectorTip, Vec2 const& sectorForwardNormal, float sectorApertureDegrees, float sectorRadius, Rgba8 const& color)
{
	constexpr int NUM_BONE_TRI = 32;
	constexpr int NUM_BONE_VERT = 3 * NUM_BONE_TRI;

	float sectorForwardDegrees = sectorForwardNormal.GetNormalized().GetOrientationDegrees();

	for (int index = 0; index < NUM_BONE_VERT; index++)
	{
		float theta = 360.0f / static_cast<float>(NUM_BONE_TRI);

		float angle1 = theta * index;
		float angle2 = theta * (index + 1);

		if (angle2 <= sectorForwardDegrees + (sectorApertureDegrees * 0.5f))
		{
			Vec2 vert1 = Vec2(sectorRadius * CosDegrees(angle1), sectorRadius * SinDegrees(angle1));
			Vec2 vert2 = Vec2(sectorRadius * CosDegrees(angle2), sectorRadius * SinDegrees(angle2));

			vert1 += sectorTip;
			vert2 += sectorTip;

			vertices.push_back(Vertex_PCU(Vec3(sectorTip.x, sectorTip.y, 0.0f), color, Vec2(0.0f, 0.0f)));
			vertices.push_back(Vertex_PCU(Vec3(vert1.x, vert1.y, 0.0f), color, Vec2(0.0f, 0.0f)));
			vertices.push_back(Vertex_PCU(Vec3(vert2.x, vert2.y, 0.0f), color, Vec2(0.0f, 0.0f)));
		}

		if (angle2 > 360.0f - sectorForwardDegrees - (sectorApertureDegrees * 0.5f) && angle2 <= 360.0f)
		{
			Vec2 vert1 = Vec2(sectorRadius * CosDegrees(angle1), sectorRadius * SinDegrees(angle1));
			Vec2 vert2 = Vec2(sectorRadius * CosDegrees(angle2), sectorRadius * SinDegrees(angle2));

			vert1 += sectorTip;
			vert2 += sectorTip;

			vertices.push_back(Vertex_PCU(Vec3(sectorTip.x, sectorTip.y, 0.0f), color, Vec2(0.0f, 0.0f)));
			vertices.push_back(Vertex_PCU(Vec3(vert1.x, vert1.y, 0.0f), color, Vec2(0.0f, 0.0f)));
			vertices.push_back(Vertex_PCU(Vec3(vert2.x, vert2.y, 0.0f), color, Vec2(0.0f, 0.0f)));
		}
	}
}

void AddVertsForOrientedSector2D(std::vector<Vertex_PCU>& vertices, Vec2 const& sectorTip, float sectorForwardDegrees, float sectorApertureDegrees, float sectorRadius, Rgba8 const& color)
{
	constexpr int NUM_BONE_TRI = 32;
	constexpr int NUM_BONE_VERT = 3 * NUM_BONE_TRI;

	for (int index = 0; index < NUM_BONE_VERT; index++)
	{
		float theta = 360.0f / static_cast<float>(NUM_BONE_TRI);

		float angle1 = theta * index;
		float angle2 = theta * (index + 1);

		if (angle2 <= sectorForwardDegrees + (sectorApertureDegrees * 0.5f))
		{
			Vec2 vert1 = Vec2(sectorRadius * CosDegrees(angle1), sectorRadius * SinDegrees(angle1));
			Vec2 vert2 = Vec2(sectorRadius * CosDegrees(angle2), sectorRadius * SinDegrees(angle2));
		
			vert1 += sectorTip;
			vert2 += sectorTip;
		
			vertices.push_back(Vertex_PCU(Vec3(sectorTip.x, sectorTip.y, 0.0f), color, Vec2(0.0f, 0.0f)));
			vertices.push_back(Vertex_PCU(Vec3(vert1.x, vert1.y, 0.0f), color, Vec2(0.0f, 0.0f)));
			vertices.push_back(Vertex_PCU(Vec3(vert2.x, vert2.y, 0.0f), color, Vec2(0.0f, 0.0f)));
		}

		if (angle2 > 360.0f - sectorForwardDegrees - (sectorApertureDegrees * 0.5f) && angle2 <= 360.0f)
		{
			Vec2 vert1 = Vec2(sectorRadius * CosDegrees(angle1), sectorRadius * SinDegrees(angle1));
			Vec2 vert2 = Vec2(sectorRadius * CosDegrees(angle2), sectorRadius * SinDegrees(angle2));
		
			vert1 += sectorTip;
			vert2 += sectorTip;
		
			vertices.push_back(Vertex_PCU(Vec3(sectorTip.x, sectorTip.y, 0.0f), color, Vec2(0.0f, 0.0f)));
			vertices.push_back(Vertex_PCU(Vec3(vert1.x, vert1.y, 0.0f), color, Vec2(0.0f, 0.0f)));
			vertices.push_back(Vertex_PCU(Vec3(vert2.x, vert2.y, 0.0f), color, Vec2(0.0f, 0.0f)));
		}
	}
}

void AddVertsForCapsule2D(std::vector<Vertex_PCU>& vertices, Vec2 const& boneStart, Vec2 const& boneEnd, float radius, Rgba8 const& color)
{
	constexpr int NUM_BONE_TRI = 32;
	constexpr int NUM_BONE_VERT = 3 * NUM_BONE_TRI;

	Vec2 direction = boneEnd - boneStart;
	float directionOrientation = direction.GetOrientationDegrees();

	for (int index = 0; index < NUM_BONE_VERT; index++)
	{
		float theta = 360.0f / static_cast<float>(NUM_BONE_TRI);

		float angle1 = directionOrientation + (theta * index);
		float angle2 = directionOrientation + (theta * (index + 1));

		if (angle2 <= directionOrientation + 90.0f)
		{

			Vec2 vert1 = Vec2(radius * CosDegrees(angle1), radius * SinDegrees(angle1));
			Vec2 vert2 = Vec2(radius * CosDegrees(angle2), radius * SinDegrees(angle2));

			vert1 += boneEnd;
			vert2 += boneEnd;

			vertices.push_back(Vertex_PCU(Vec3(boneEnd.x, boneEnd.y, 0.0f), color, Vec2(0.0f, 0.0f)));
			vertices.push_back(Vertex_PCU(Vec3(vert1.x, vert1.y, 0.0f), color, Vec2(0.0f, 0.0f)));
			vertices.push_back(Vertex_PCU(Vec3(vert2.x, vert2.y, 0.0f), color, Vec2(0.0f, 0.0f)));
		}

		if (angle2 > directionOrientation + 270.0f && angle2 <= directionOrientation + 360.0f)
		{
			Vec2 vert1 = Vec2(radius * CosDegrees(angle1), radius * SinDegrees(angle1));
			Vec2 vert2 = Vec2(radius * CosDegrees(angle2), radius * SinDegrees(angle2));

			vert1 += boneEnd;
			vert2 += boneEnd;

			vertices.push_back(Vertex_PCU(Vec3(boneEnd.x, boneEnd.y, 0.0f), color, Vec2(0.0f, 0.0f)));
			vertices.push_back(Vertex_PCU(Vec3(vert1.x, vert1.y, 0.0f), color, Vec2(0.0f, 0.0f)));
			vertices.push_back(Vertex_PCU(Vec3(vert2.x, vert2.y, 0.0f), color, Vec2(0.0f, 0.0f)));
		}

		if (angle2 > directionOrientation + 90.0f && angle2 <= directionOrientation + 270.0f)
		{
			Vec2 vert1 = Vec2(radius * CosDegrees(angle1), radius * SinDegrees(angle1));
			Vec2 vert2 = Vec2(radius * CosDegrees(angle2), radius * SinDegrees(angle2));

			vert1 += boneStart;
			vert2 += boneStart;

			vertices.push_back(Vertex_PCU(Vec3(boneStart.x, boneStart.y, 0.0f), color, Vec2(0.0f, 0.0f)));
			vertices.push_back(Vertex_PCU(Vec3(vert1.x, vert1.y, 0.0f), color, Vec2(0.0f, 0.0f)));
			vertices.push_back(Vertex_PCU(Vec3(vert2.x, vert2.y, 0.0f), color, Vec2(0.0f, 0.0f)));
		}
	}

	direction.Normalize();
	direction.SetLength(radius);
	direction.Rotate90Degrees();

	Vec2 startLeft = boneStart + direction;
	Vec2 startRight = boneStart - direction;
	Vec2 endLeft = boneEnd + direction;
	Vec2 endRight = boneEnd - direction;

	vertices.push_back(Vertex_PCU(Vec3(startLeft.x, startLeft.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	vertices.push_back(Vertex_PCU(Vec3(startRight.x, startRight.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	vertices.push_back(Vertex_PCU(Vec3(endRight.x, endRight.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	vertices.push_back(Vertex_PCU(Vec3(endRight.x, endRight.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	vertices.push_back(Vertex_PCU(Vec3(endLeft.x, endLeft.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	vertices.push_back(Vertex_PCU(Vec3(startLeft.x, startLeft.y, 0.0f), color, Vec2(0.0f, 0.0f)));
}

void AddVertsForDisc2D(std::vector<Vertex_PCU>& vertices, Vec2 const& center, float radius, Rgba8 const& color)
{
	constexpr int NUM_BONE_TRI = 64;
	//constexpr int NUM_BONE_VERT = 3 * NUM_BONE_TRI;

	for (int index = 0; index < NUM_BONE_TRI; index++)
	{
		float theta = 360.0f / static_cast<float>(NUM_BONE_TRI);

		float angle1 = theta * index;
		float angle2 = theta * (index + 1);
			Vec2 vert1 = Vec2(radius * CosDegrees(angle1), radius * SinDegrees(angle1));
			Vec2 vert2 = Vec2(radius * CosDegrees(angle2), radius * SinDegrees(angle2));

			vert1 += center;
			vert2 += center;

			vertices.push_back(Vertex_PCU(Vec3(center.x, center.y, 0.0f), color, Vec2(0.0f, 0.0f)));
			vertices.push_back(Vertex_PCU(Vec3(vert1.x, vert1.y, 0.0f), color, Vec2(0.0f, 0.0f)));
			vertices.push_back(Vertex_PCU(Vec3(vert2.x, vert2.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	}
}

void AddVertsForAABB2D(std::vector<Vertex_PCU>& vertices, AABB2 const& bounds, Rgba8 const& color, Vec2 uvMins, Vec2 uvMaxs)
{
	vertices.push_back(Vertex_PCU(Vec3(bounds.m_mins.x, bounds.m_mins.y, 0.0f), color, uvMins)); //1
	vertices.push_back(Vertex_PCU(Vec3(bounds.m_maxs.x, bounds.m_mins.y, 0.0f), color, Vec2(uvMaxs.x, uvMins.y))); //2
	vertices.push_back(Vertex_PCU(Vec3(bounds.m_maxs.x, bounds.m_maxs.y, 0.0f), color, uvMaxs)); //3
	vertices.push_back(Vertex_PCU(Vec3(bounds.m_maxs.x, bounds.m_maxs.y, 0.0f), color, uvMaxs)); //3
	vertices.push_back(Vertex_PCU(Vec3(bounds.m_mins.x, bounds.m_maxs.y, 0.0f), color, Vec2(uvMins.x, uvMaxs.y))); //4
	vertices.push_back(Vertex_PCU(Vec3(bounds.m_mins.x, bounds.m_mins.y, 0.0f), color, uvMins)); //1
}

void AddVertsForOBB2D(std::vector<Vertex_PCU>& vertices, OBB2 const& box, Rgba8 const& color)
{
	Vec2 iDirection = box.m_iBasisNormal.GetNormalized();
	Vec2 jDirection = iDirection.GetRotated90Degrees();
	iDirection.SetLength(box.m_halfDimensions.x);
	jDirection.SetLength(box.m_halfDimensions.y);

	Vec2 startLeft = box.m_center - iDirection - jDirection;
	Vec2 startRight = box.m_center + iDirection - jDirection;
	Vec2 endLeft = box.m_center - iDirection + jDirection;
	Vec2 endRight = box.m_center + iDirection + jDirection;

	vertices.push_back(Vertex_PCU(Vec3(startLeft.x, startLeft.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	vertices.push_back(Vertex_PCU(Vec3(startRight.x, startRight.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	vertices.push_back(Vertex_PCU(Vec3(endRight.x, endRight.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	vertices.push_back(Vertex_PCU(Vec3(endRight.x, endRight.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	vertices.push_back(Vertex_PCU(Vec3(endLeft.x, endLeft.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	vertices.push_back(Vertex_PCU(Vec3(startLeft.x, startLeft.y, 0.0f), color, Vec2(0.0f, 0.0f)));
}

void AddVertsForOBB3D(std::vector<Vertex_PCU>& vertices, OBB3 const& box, Rgba8 const& color, AABB2 const& UVs)
{
	Vec3 iBasis = box.m_iBasisNormal.GetNormalized();
	Vec3 jBasis = CrossProduct3D(Vec3(0.0f, 0.0f, 1.0f), iBasis).GetNormalized();
	Vec3 kBasis = CrossProduct3D(jBasis, iBasis).GetNormalized();

	Vec3 vertices1 = box.m_center + (box.m_halfDimensions.x * iBasis) + (box.m_halfDimensions.y * jBasis) + (box.m_halfDimensions.z * kBasis);
	Vec3 vertices2 = box.m_center + (box.m_halfDimensions.x * iBasis) - (box.m_halfDimensions.y * jBasis) + (box.m_halfDimensions.z * kBasis);
	Vec3 vertices3 = box.m_center + (box.m_halfDimensions.x * iBasis) - (box.m_halfDimensions.y * jBasis) - (box.m_halfDimensions.z * kBasis);
	Vec3 vertices4 = box.m_center + (box.m_halfDimensions.x * iBasis) + (box.m_halfDimensions.y * jBasis) - (box.m_halfDimensions.z * kBasis);
	Vec3 vertices5 = box.m_center - (box.m_halfDimensions.x * iBasis) + (box.m_halfDimensions.y * jBasis) + (box.m_halfDimensions.z * kBasis);
	Vec3 vertices6 = box.m_center - (box.m_halfDimensions.x * iBasis) - (box.m_halfDimensions.y * jBasis) + (box.m_halfDimensions.z * kBasis);
	Vec3 vertices7 = box.m_center - (box.m_halfDimensions.x * iBasis) - (box.m_halfDimensions.y * jBasis) - (box.m_halfDimensions.z * kBasis);
	Vec3 vertices8 = box.m_center - (box.m_halfDimensions.x * iBasis) + (box.m_halfDimensions.y * jBasis) - (box.m_halfDimensions.z * kBasis);

	AddVertsForQuad3D(vertices, vertices4, vertices3, vertices2, vertices1, color, color, UVs);
	AddVertsForQuad3D(vertices, vertices8, vertices4, vertices1, vertices5, color, color, UVs);
	AddVertsForQuad3D(vertices, vertices7, vertices8, vertices5, vertices6, color, color, UVs);
	AddVertsForQuad3D(vertices, vertices3, vertices7, vertices6, vertices2, color, color, UVs);
	AddVertsForQuad3D(vertices, vertices1, vertices2, vertices6, vertices5, color, color, UVs);
	AddVertsForQuad3D(vertices, vertices8, vertices7, vertices3, vertices4, color, color, UVs);
}

void AddVertsForLineSegment2D(std::vector<Vertex_PCU>& vertices, Vec2 const& start, Vec2 const& end, float thickness, Rgba8 const& color)
{
	Vec2 direction = end - start;
	direction.Normalize();
	direction.SetLength(thickness * 0.5f);
	direction.Rotate90Degrees();

	Vec2 startLeft		= start + direction;
	Vec2 startRight		= start - direction;
	Vec2 endLeft		= end	+ direction;
	Vec2 endRight		= end	- direction;

	vertices.push_back(Vertex_PCU(Vec3(startLeft.x, startLeft.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	vertices.push_back(Vertex_PCU(Vec3(startRight.x, startRight.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	vertices.push_back(Vertex_PCU(Vec3(endRight.x, endRight.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	vertices.push_back(Vertex_PCU(Vec3(endRight.x, endRight.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	vertices.push_back(Vertex_PCU(Vec3(endLeft.x, endLeft.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	vertices.push_back(Vertex_PCU(Vec3(startLeft.x, startLeft.y, 0.0f), color, Vec2(0.0f, 0.0f)));
}

void AddVertsForArrow2D(std::vector<Vertex_PCU>& vertices, Vec2 tailPos, Vec2 tipPos, float arrowSize, float lineThickness, Rgba8 const& color)
{
	AddVertsForLineSegment2D(vertices, tipPos, tailPos, lineThickness, color);

	Vec2 fwdNml = (tipPos - tailPos).GetNormalized();
	fwdNml.SetLength(arrowSize);
	Vec2 leftHook = Vec2::MakeFromPolarDegrees(fwdNml.GetOrientationDegrees() + 135.0f, fwdNml.GetLength());
	Vec2 rightHook = Vec2::MakeFromPolarDegrees(fwdNml.GetOrientationDegrees() - 135.0f, fwdNml.GetLength());

	leftHook += tipPos;
	rightHook += tipPos;

	AddVertsForLineSegment2D(vertices, tipPos, leftHook, lineThickness, color);
	AddVertsForLineSegment2D(vertices, tipPos, rightHook, lineThickness, color);
}

void AddVertsForQuad3D(std::vector<Vertex_PCU>& vertices, Vec3 const& bottomLeft, Vec3 const& bottomRight, Vec3 const& topRight, Vec3 const& topLeft, Rgba8 const& color, AABB2 const& UVs)
{
	vertices.push_back(Vertex_PCU(bottomLeft,	color, UVs.m_mins));
	vertices.push_back(Vertex_PCU(bottomRight,	color, Vec2(UVs.m_maxs.x, UVs.m_mins.y)));
	vertices.push_back(Vertex_PCU(topRight,		color, UVs.m_maxs));
	vertices.push_back(Vertex_PCU(topRight,		color, UVs.m_maxs));
	vertices.push_back(Vertex_PCU(topLeft,		color, Vec2(UVs.m_mins.x, UVs.m_maxs.y)));
	vertices.push_back(Vertex_PCU(bottomLeft,	color, UVs.m_mins));
}

void AddVertsForQuad3D(std::vector<Vertex_PCU>& vertices, Vec3 const& bottomLeft, Vec3 const& bottomRight, Vec3 const& topRight, Vec3 const& topLeft, Rgba8 const& startColor, Rgba8 const& endColor, AABB2 const& UVs)
{
	vertices.push_back(Vertex_PCU(bottomLeft, startColor, UVs.m_mins));
	vertices.push_back(Vertex_PCU(bottomRight, startColor, Vec2(UVs.m_maxs.x, UVs.m_mins.y)));
	vertices.push_back(Vertex_PCU(topRight, endColor, UVs.m_maxs));
	vertices.push_back(Vertex_PCU(topRight, endColor, UVs.m_maxs));
	vertices.push_back(Vertex_PCU(topLeft, endColor, Vec2(UVs.m_mins.x, UVs.m_maxs.y)));
	vertices.push_back(Vertex_PCU(bottomLeft, startColor, UVs.m_mins));
}

void AddVertsForQuad3D(std::vector<Vertex_PCUTBN>& vertices, Vec3 const& bottomLeft, Vec3 const& bottomRight, Vec3 const& topRight, Vec3 const& topLeft, Rgba8 const& color, AABB2 const& UVs)
{
	Vec3 quadTangent1 = bottomRight - bottomLeft;
	Vec3 quadTangent2 = topLeft - bottomLeft;
	Vec3 quadNormal = CrossProduct3D(quadTangent1, quadTangent2).GetNormalized();

	vertices.push_back(Vertex_PCUTBN(bottomLeft, color, UVs.m_mins, Vec3::ZERO, Vec3::ZERO, quadNormal));
	vertices.push_back(Vertex_PCUTBN(bottomRight, color, Vec2(UVs.m_maxs.x, UVs.m_mins.y), Vec3::ZERO, Vec3::ZERO, quadNormal));
	vertices.push_back(Vertex_PCUTBN(topRight, color, UVs.m_maxs, Vec3::ZERO, Vec3::ZERO, quadNormal));
	vertices.push_back(Vertex_PCUTBN(topRight, color, UVs.m_maxs, Vec3::ZERO, Vec3::ZERO, quadNormal));
	vertices.push_back(Vertex_PCUTBN(topLeft, color, Vec2(UVs.m_mins.x, UVs.m_maxs.y), Vec3::ZERO, Vec3::ZERO, quadNormal));
	vertices.push_back(Vertex_PCUTBN(bottomLeft, color, UVs.m_mins, Vec3::ZERO, Vec3::ZERO, quadNormal));
}

void AddVertsForQuad3D(std::vector<Vertex_PCU>& vertices, std::vector<unsigned int>& indices, int indexOffset, Vec3 const& bottomLeft, Vec3 const& bottomRight, Vec3 const& topRight, Vec3 const& topLeft, Rgba8 const& color, AABB2 const& UVs)
{
	vertices.push_back(Vertex_PCU(bottomLeft, color, UVs.m_mins));    //0
	vertices.push_back(Vertex_PCU(bottomRight, color, Vec2(UVs.m_maxs.x, UVs.m_mins.y))); //1
	vertices.push_back(Vertex_PCU(topRight, color, UVs.m_maxs)); //2
	vertices.push_back(Vertex_PCU(topLeft, color, Vec2(UVs.m_mins.x, UVs.m_maxs.y)));//3

	indices.push_back(0 + indexOffset);
	indices.push_back(1 + indexOffset);
	indices.push_back(2 + indexOffset);
	indices.push_back(2 + indexOffset);
	indices.push_back(3 + indexOffset);
	indices.push_back(0 + indexOffset);
}

void AddVertsForQuad3D(std::vector<Vertex_PCUTBN>& vertices, std::vector<unsigned int>& indices, int indexOffset, Vec3 const& bottomLeft, Vec3 const& bottomRight, Vec3 const& topRight, Vec3 const& topLeft, Rgba8 const& color, AABB2 const& UVs)
{
	Vec3 quadTangent1 = bottomRight - bottomLeft;
	Vec3 quadTangent2 = topLeft - bottomLeft;
	Vec3 quadNormal = CrossProduct3D(quadTangent1, quadTangent2).GetNormalized();
	Vec3 quadTangent;
	Vec3 quadBiTangent;

	if (quadNormal == Vec3(0.0f, 0.0f, 1.0f))
	{
		quadTangent = Vec3(1.0f, 0.0f, 0.0f);
		quadBiTangent = Vec3(0.0f, 1.0f, 0.0f);
	}
	else if (quadNormal == Vec3(0.0f, 0.0f, -1.0f))
	{
		quadTangent = Vec3(-1.0f, 0.0f, 0.0f);
		quadBiTangent = Vec3(0.0f, -1.0f, 0.0f);
	}
	else
	{
		quadTangent = CrossProduct3D(Vec3(0.0f, 0.0f, 1.0f), quadNormal).GetNormalized();
		quadBiTangent = CrossProduct3D(quadNormal, quadTangent).GetNormalized();
	}

	vertices.push_back(Vertex_PCUTBN(bottomLeft, color, UVs.m_mins, quadTangent, quadBiTangent, quadNormal));
	vertices.push_back(Vertex_PCUTBN(bottomRight, color, Vec2(UVs.m_maxs.x, UVs.m_mins.y), quadTangent, quadBiTangent, quadNormal));
	vertices.push_back(Vertex_PCUTBN(topRight, color, UVs.m_maxs, quadTangent, quadBiTangent, quadNormal));
	vertices.push_back(Vertex_PCUTBN(topLeft, color, Vec2(UVs.m_mins.x, UVs.m_maxs.y), quadTangent, quadBiTangent, quadNormal));

	indices.push_back(0 + indexOffset);
	indices.push_back(1 + indexOffset);
	indices.push_back(2 + indexOffset);
	indices.push_back(2 + indexOffset);
	indices.push_back(3 + indexOffset);
	indices.push_back(0 + indexOffset);
}

void AddVertsForAABB3D(std::vector<Vertex_PCU>& vertices, AABB3 const& bounds, Rgba8 const& color, AABB2 const& UVs)
{
	Vec3 bottomLeftNear		= Vec3(bounds.m_maxs.x, bounds.m_mins.y, bounds.m_mins.z);
	Vec3 bottomRightNear	= Vec3(bounds.m_maxs.x, bounds.m_maxs.y, bounds.m_mins.z);
	Vec3 topLeftNear		= Vec3(bounds.m_maxs.x, bounds.m_mins.y, bounds.m_maxs.z);
	Vec3 topRightNear		= Vec3(bounds.m_maxs.x, bounds.m_maxs.y, bounds.m_maxs.z);
	Vec3 bottomLeftFar		= Vec3(bounds.m_mins.x, bounds.m_mins.y, bounds.m_mins.z);
	Vec3 bottomRightFar		= Vec3(bounds.m_mins.x, bounds.m_maxs.y, bounds.m_mins.z);
	Vec3 topLeftFar			= Vec3(bounds.m_mins.x, bounds.m_mins.y, bounds.m_maxs.z);
	Vec3 topRightFar		= Vec3(bounds.m_mins.x, bounds.m_maxs.y, bounds.m_maxs.z);

	AddVertsForQuad3D(vertices, bottomLeftNear,		bottomRightNear,	topRightNear,		topLeftNear,		color, UVs);
	AddVertsForQuad3D(vertices, bottomRightNear,	bottomRightFar,		topRightFar,		topRightNear,		color, UVs);
	AddVertsForQuad3D(vertices, bottomRightFar,		bottomLeftFar,		topLeftFar,			topRightFar,		color, UVs);
	AddVertsForQuad3D(vertices, bottomLeftFar,		bottomLeftNear,		topLeftNear,		topLeftFar,			color, UVs);
	AddVertsForQuad3D(vertices, topLeftNear,		topRightNear,		topRightFar,		topLeftFar,			color, UVs);
	AddVertsForQuad3D(vertices, bottomLeftFar,		bottomRightFar,		bottomRightNear,	bottomLeftNear,		color, UVs);
}

void AddVertsForAABB3D(std::vector<Vertex_PCU>& vertices, std::vector<unsigned int>& indices, int indexOffset, AABB3 const& bounds, Rgba8 const& color, AABB2 const& UVs)
{
	Vec3 bottomLeftNear				= Vec3(bounds.m_maxs.x, bounds.m_mins.y, bounds.m_mins.z);
	Vec3 bottomRightNear			= Vec3(bounds.m_maxs.x, bounds.m_maxs.y, bounds.m_mins.z);
	Vec3 topLeftNear				= Vec3(bounds.m_maxs.x, bounds.m_mins.y, bounds.m_maxs.z);
	Vec3 topRightNear				= Vec3(bounds.m_maxs.x, bounds.m_maxs.y, bounds.m_maxs.z);
	Vec3 bottomLeftFar				= Vec3(bounds.m_mins.x, bounds.m_mins.y, bounds.m_mins.z);
	Vec3 bottomRightFar				= Vec3(bounds.m_mins.x, bounds.m_maxs.y, bounds.m_mins.z);
	Vec3 topLeftFar					= Vec3(bounds.m_mins.x, bounds.m_mins.y, bounds.m_maxs.z);
	Vec3 topRightFar				= Vec3(bounds.m_mins.x, bounds.m_maxs.y, bounds.m_maxs.z);

	AddVertsForQuad3D(vertices, indices, indexOffset, bottomLeftNear,	bottomRightNear,	topRightNear,		topLeftNear,	color, UVs);
	indexOffset += 4;

	AddVertsForQuad3D(vertices, indices, indexOffset, bottomRightNear,	bottomRightFar,		topRightFar,		topRightNear,	color, UVs);
	indexOffset += 4;

	AddVertsForQuad3D(vertices, indices, indexOffset, bottomRightFar,	bottomLeftFar,		topLeftFar,			topRightFar,	color, UVs);
	indexOffset += 4;

	AddVertsForQuad3D(vertices, indices, indexOffset, bottomLeftFar,	bottomLeftNear,		topLeftNear,		topLeftFar,		color, UVs);
	indexOffset += 4;

	AddVertsForQuad3D(vertices, indices, indexOffset, topLeftNear,		topRightNear,		topRightFar,		topLeftFar,		color, UVs);
	indexOffset += 4;

	AddVertsForQuad3D(vertices, indices, indexOffset, bottomLeftFar,	bottomRightFar,		bottomRightNear,	bottomLeftNear, color, UVs);
	indexOffset += 4;
}

void AddVertsForAABB3D(std::vector<Vertex_PCUTBN>& vertices, AABB3 const& bounds, Rgba8 const& color, AABB2 const& UVs)
{
	Vec3 bottomLeftNear = Vec3(bounds.m_maxs.x, bounds.m_mins.y, bounds.m_mins.z);
	Vec3 bottomRightNear = Vec3(bounds.m_maxs.x, bounds.m_maxs.y, bounds.m_mins.z);
	Vec3 topLeftNear = Vec3(bounds.m_maxs.x, bounds.m_mins.y, bounds.m_maxs.z);
	Vec3 topRightNear = Vec3(bounds.m_maxs.x, bounds.m_maxs.y, bounds.m_maxs.z);
	Vec3 bottomLeftFar = Vec3(bounds.m_mins.x, bounds.m_mins.y, bounds.m_mins.z);
	Vec3 bottomRightFar = Vec3(bounds.m_mins.x, bounds.m_maxs.y, bounds.m_mins.z);
	Vec3 topLeftFar = Vec3(bounds.m_mins.x, bounds.m_mins.y, bounds.m_maxs.z);
	Vec3 topRightFar = Vec3(bounds.m_mins.x, bounds.m_maxs.y, bounds.m_maxs.z);

	AddVertsForQuad3D(vertices, bottomLeftNear, bottomRightNear, topRightNear, topLeftNear, color, UVs);
	AddVertsForQuad3D(vertices, bottomRightNear, bottomRightFar, topRightFar, topRightNear, color, UVs);
	AddVertsForQuad3D(vertices, bottomRightFar, bottomLeftFar, topLeftFar, topRightFar, color, UVs);
	AddVertsForQuad3D(vertices, bottomLeftFar, bottomLeftNear, topLeftNear, topLeftFar, color, UVs);
	AddVertsForQuad3D(vertices, topLeftNear, topRightNear, topRightFar, topLeftFar, color, UVs);
	AddVertsForQuad3D(vertices, bottomLeftFar, bottomRightFar, bottomRightNear, bottomLeftNear, color, UVs);
}

void AddVertsForAABB3D(std::vector<Vertex_PCUTBN>& vertices, std::vector<unsigned int>& indices, int indexOffset, AABB3 const& bounds, Rgba8 const& color, AABB2 const& UVs)
{
	Vec3 bottomLeftNear				= Vec3(bounds.m_maxs.x, bounds.m_mins.y, bounds.m_mins.z);
	Vec3 bottomRightNear			= Vec3(bounds.m_maxs.x, bounds.m_maxs.y, bounds.m_mins.z);
	Vec3 topLeftNear				= Vec3(bounds.m_maxs.x, bounds.m_mins.y, bounds.m_maxs.z);
	Vec3 topRightNear				= Vec3(bounds.m_maxs.x, bounds.m_maxs.y, bounds.m_maxs.z);
	Vec3 bottomLeftFar				= Vec3(bounds.m_mins.x, bounds.m_mins.y, bounds.m_mins.z);
	Vec3 bottomRightFar				= Vec3(bounds.m_mins.x, bounds.m_maxs.y, bounds.m_mins.z);
	Vec3 topLeftFar					= Vec3(bounds.m_mins.x, bounds.m_mins.y, bounds.m_maxs.z);
	Vec3 topRightFar				= Vec3(bounds.m_mins.x, bounds.m_maxs.y, bounds.m_maxs.z);

	AddVertsForQuad3D(vertices, indices, indexOffset, bottomLeftNear,	bottomRightNear,	topRightNear,		topLeftNear,	color, UVs);
	indexOffset += 4;

	AddVertsForQuad3D(vertices, indices, indexOffset, bottomRightNear,	bottomRightFar,		topRightFar,		topRightNear,	color, UVs);
	indexOffset += 4;

	AddVertsForQuad3D(vertices, indices, indexOffset, bottomRightFar,	bottomLeftFar,		topLeftFar,			topRightFar,	color, UVs);
	indexOffset += 4;

	AddVertsForQuad3D(vertices, indices, indexOffset, bottomLeftFar,	bottomLeftNear,		topLeftNear,		topLeftFar,		color, UVs);
	indexOffset += 4;

	AddVertsForQuad3D(vertices, indices, indexOffset, topLeftNear,		topRightNear,		topRightFar,		topLeftFar,		color, UVs);
	indexOffset += 4;

	AddVertsForQuad3D(vertices, indices, indexOffset, bottomLeftFar,	bottomRightFar,		bottomRightNear,	bottomLeftNear, color, UVs);
	indexOffset += 4;
}

void AddVertsForPlane3D(std::vector<Vertex_PCU>& vertices, Plane3D const& plane)
{
	

	Vec3 iBasis = plane.m_normal.GetNormalized();
	Vec3 jBasis = CrossProduct3D(Vec3(0.0f, 0.0f, 1.0f), iBasis).GetNormalized();
	Vec3 kBasis = CrossProduct3D(jBasis, iBasis).GetNormalized();
	Vec3 planeCenter = plane.m_normal * plane.m_distanceFromOriginAlongNormal;

	float LineSpace = 1.0f;

	Vec3 verticalPoint1 = planeCenter + (kBasis * LineSpace);
	Vec3 verticalPoint2 = planeCenter + (kBasis * 2.0f * LineSpace);
	Vec3 verticalPoint3 = planeCenter + (kBasis * 3.0f * LineSpace);
	Vec3 verticalPoint4 = planeCenter + (kBasis * -1.0f * LineSpace);
	Vec3 verticalPoint5 = planeCenter + (kBasis * -2.0f * LineSpace);
	Vec3 verticalPoint6 = planeCenter + (kBasis * -3.0f *LineSpace);

	AddVertsForCylinder3D(vertices, planeCenter, planeCenter + (jBasis * 10.0f), 0.025f, Rgba8::RED, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, planeCenter, planeCenter - (jBasis * 10.0f), 0.025f, Rgba8::RED, Rgba8(0, 0, 0, 0));

	AddVertsForCylinder3D(vertices, verticalPoint1, verticalPoint1 + (jBasis * 10.0f), 0.025f, Rgba8::RED, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, verticalPoint2, verticalPoint2 + (jBasis * 10.0f), 0.025f, Rgba8::RED, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, verticalPoint3, verticalPoint3 + (jBasis * 10.0f), 0.025f, Rgba8::RED, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, verticalPoint4, verticalPoint4 + (jBasis * 10.0f), 0.025f, Rgba8::RED, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, verticalPoint5, verticalPoint5 + (jBasis * 10.0f), 0.025f, Rgba8::RED, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, verticalPoint6, verticalPoint6 + (jBasis * 10.0f), 0.025f, Rgba8::RED, Rgba8(0, 0, 0, 0));
																				
	AddVertsForCylinder3D(vertices, verticalPoint1, verticalPoint1 - (jBasis * 10.0f), 0.025f, Rgba8::RED, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, verticalPoint2, verticalPoint2 - (jBasis * 10.0f), 0.025f, Rgba8::RED, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, verticalPoint3, verticalPoint3 - (jBasis * 10.0f), 0.025f, Rgba8::RED, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, verticalPoint4, verticalPoint4 - (jBasis * 10.0f), 0.025f, Rgba8::RED, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, verticalPoint5, verticalPoint5 - (jBasis * 10.0f), 0.025f, Rgba8::RED, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, verticalPoint6, verticalPoint6 - (jBasis * 10.0f), 0.025f, Rgba8::RED, Rgba8(0, 0, 0, 0));

	Vec3 horizontalPoint1 = planeCenter + (jBasis * LineSpace);
	Vec3 horizontalPoint2 = planeCenter + (jBasis * 2.0f * LineSpace);
	Vec3 horizontalPoint3 = planeCenter + (jBasis * 3.0f * LineSpace);
	Vec3 horizontalPoint4 = planeCenter + (jBasis * -1.0f * LineSpace);
	Vec3 horizontalPoint5 = planeCenter + (jBasis * -2.0f * LineSpace);
	Vec3 horizontalPoint6 = planeCenter + (jBasis * -3.0f * LineSpace);

	AddVertsForArrow(vertices, planeCenter, planeCenter + iBasis, 0.025f, Rgba8::WHITE);
	AddVertsForArrow(vertices, horizontalPoint1, horizontalPoint1 + iBasis, 0.025f, Rgba8::WHITE);
	AddVertsForArrow(vertices, horizontalPoint2, horizontalPoint2 + iBasis, 0.025f, Rgba8::WHITE);
	AddVertsForArrow(vertices, horizontalPoint3, horizontalPoint3 + iBasis, 0.025f, Rgba8::WHITE);
	AddVertsForArrow(vertices, horizontalPoint4, horizontalPoint4 + iBasis, 0.025f, Rgba8::WHITE);
	AddVertsForArrow(vertices, horizontalPoint5, horizontalPoint5 + iBasis, 0.025f, Rgba8::WHITE);
	AddVertsForArrow(vertices, horizontalPoint6, horizontalPoint6 + iBasis, 0.025f, Rgba8::WHITE);


	AddVertsForCylinder3D(vertices, planeCenter, planeCenter + (kBasis * 10.0f), 0.025f, Rgba8::GREEN, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, planeCenter, planeCenter - (kBasis * 10.0f), 0.025f, Rgba8::GREEN, Rgba8(0, 0, 0, 0));

	AddVertsForCylinder3D(vertices, horizontalPoint1, horizontalPoint1 + (kBasis * 10.0f), 0.025f, Rgba8::GREEN, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, horizontalPoint2, horizontalPoint2 + (kBasis * 10.0f), 0.025f, Rgba8::GREEN, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, horizontalPoint3, horizontalPoint3 + (kBasis * 10.0f), 0.025f, Rgba8::GREEN, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, horizontalPoint4, horizontalPoint4 + (kBasis * 10.0f), 0.025f, Rgba8::GREEN, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, horizontalPoint5, horizontalPoint5 + (kBasis * 10.0f), 0.025f, Rgba8::GREEN, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, horizontalPoint6, horizontalPoint6 + (kBasis * 10.0f), 0.025f, Rgba8::GREEN, Rgba8(0, 0, 0, 0));
																					
	AddVertsForCylinder3D(vertices, horizontalPoint1, horizontalPoint1 - (kBasis * 10.0f), 0.025f, Rgba8::GREEN, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, horizontalPoint2, horizontalPoint2 - (kBasis * 10.0f), 0.025f, Rgba8::GREEN, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, horizontalPoint3, horizontalPoint3 - (kBasis * 10.0f), 0.025f, Rgba8::GREEN, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, horizontalPoint4, horizontalPoint4 - (kBasis * 10.0f), 0.025f, Rgba8::GREEN, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, horizontalPoint5, horizontalPoint5 - (kBasis * 10.0f), 0.025f, Rgba8::GREEN, Rgba8(0, 0, 0, 0));
	AddVertsForCylinder3D(vertices, horizontalPoint6, horizontalPoint6 - (kBasis * 10.0f), 0.025f, Rgba8::GREEN, Rgba8(0, 0, 0, 0));
}

void AddVertsForWireAABB3D(std::vector<Vertex_PCU>& vertices, AABB3 const& bounds, Rgba8 const& color, AABB2 const& UVs)
{
	Vec3 bottomLeftNear = Vec3(bounds.m_maxs.x, bounds.m_mins.y, bounds.m_mins.z);
	Vec3 bottomRightNear = Vec3(bounds.m_maxs.x, bounds.m_maxs.y, bounds.m_mins.z);
	Vec3 topLeftNear = Vec3(bounds.m_maxs.x, bounds.m_mins.y, bounds.m_maxs.z);
	Vec3 topRightNear = Vec3(bounds.m_maxs.x, bounds.m_maxs.y, bounds.m_maxs.z);
	Vec3 bottomLeftFar = Vec3(bounds.m_mins.x, bounds.m_mins.y, bounds.m_mins.z);
	Vec3 bottomRightFar = Vec3(bounds.m_mins.x, bounds.m_maxs.y, bounds.m_mins.z);
	Vec3 topLeftFar = Vec3(bounds.m_mins.x, bounds.m_mins.y, bounds.m_maxs.z);
	Vec3 topRightFar = Vec3(bounds.m_mins.x, bounds.m_maxs.y, bounds.m_maxs.z);

	AddVertsForCylinder3D(vertices, bottomLeftNear, bottomRightNear, 0.02f, color, UVs, 3);
	AddVertsForCylinder3D(vertices, bottomRightNear, topRightNear, 0.02f, color, UVs, 3);
	AddVertsForCylinder3D(vertices, topRightNear, topLeftNear, 0.02f, color, UVs, 3);
	AddVertsForCylinder3D(vertices, bottomLeftNear, topLeftNear, 0.02f, color, UVs, 3);
	AddVertsForCylinder3D(vertices, bottomLeftFar, bottomRightFar, 0.02f, color, UVs, 3);
	AddVertsForCylinder3D(vertices, bottomRightFar, topRightFar, 0.02f, color, UVs, 3);
	AddVertsForCylinder3D(vertices, topRightFar, topLeftFar, 0.02f, color, UVs, 3);
	AddVertsForCylinder3D(vertices, bottomLeftFar, topLeftFar, 0.02f, color, UVs, 3);
	AddVertsForCylinder3D(vertices, bottomLeftNear, bottomLeftFar, 0.02f, color, UVs, 3);
	AddVertsForCylinder3D(vertices, bottomRightNear, bottomRightFar, 0.02f, color, UVs, 3);
	AddVertsForCylinder3D(vertices, topRightNear, topRightFar, 0.02f, color, UVs, 3);
	AddVertsForCylinder3D(vertices, topLeftNear, topLeftFar, 0.02f, color, UVs, 3);
}

void AddVertsForSphere3D(std::vector<Vertex_PCU>& vertices, Vec3 const& center, float radius, Rgba8 const& color, AABB2 const& UVs, int numLatitudeSlices, int numLongitudeSlices)
{
	UNUSED(UVs);

	float yawTheta = 360.0f / (float)numLatitudeSlices;
	float pitchTheta = 360.0f / (float)numLongitudeSlices;

	for (int pitchIndex = 0; pitchIndex < numLongitudeSlices; pitchIndex++)
	{
		if (pitchTheta * (pitchIndex + 1) >= 0.0f && pitchTheta * (pitchIndex + 1) <= 90.0f + pitchTheta)
		{
			for (int yawIndex = 0; yawIndex < numLatitudeSlices; yawIndex++)
			{
				float angleleft = yawTheta * yawIndex;
				float angleRight = yawTheta + angleleft;
				float angleTop = pitchTheta * pitchIndex;
				float angleBottom = pitchTheta + angleTop;

				Vec3 bl = Vec3::MakeFromPolarDegrees(angleBottom, angleleft, radius);
				Vec3 br = Vec3::MakeFromPolarDegrees(angleBottom, angleRight, radius);
				Vec3 tl = Vec3::MakeFromPolarDegrees(angleTop, angleleft, radius);
				Vec3 tr = Vec3::MakeFromPolarDegrees(angleTop, angleRight, radius);

				bl += center;
				br += center;
				tl += center;
				tr += center;

				float uLeftTexCoord = RangeMap(angleleft, 0.0f, 360.0f, 0.0f, 1.0f);
				float uRightTexCoord = RangeMap(angleRight, 0.0f, 360.0f, 0.0f, 1.0f);

				float vBottomTexCoord = RangeMap(angleBottom, 0.0f, 90.0f, 0.5f, 0.0f);
				float vTopTexCoord = RangeMap(angleTop, 0.0f, 90.0f, 0.5f, 0.0f);

				AddVertsForQuad3D(vertices, bl, br, tr, tl, color, AABB2(uLeftTexCoord, vBottomTexCoord, uRightTexCoord, vTopTexCoord));
			}
		}

		if (pitchTheta * (pitchIndex + 1) >= 270.0f && pitchTheta * (pitchIndex + 1) <= 360.0f)
		{
			for (int yawIndex = 0; yawIndex < numLatitudeSlices; yawIndex++)
			{
				float angleleft = yawTheta * yawIndex;
				float angleRight = yawTheta + angleleft;
				float angleTop = pitchTheta * pitchIndex;
				float angleBottom = pitchTheta + angleTop;

				Vec3 bl = Vec3::MakeFromPolarDegrees(angleBottom, angleleft, radius);
				Vec3 br = Vec3::MakeFromPolarDegrees(angleBottom, angleRight, radius);
				Vec3 tl = Vec3::MakeFromPolarDegrees(angleTop, angleleft, radius);
				Vec3 tr = Vec3::MakeFromPolarDegrees(angleTop, angleRight, radius);

				bl += center;
				br += center;
				tl += center;
				tr += center;

				float uLeftTexCoord = RangeMap(angleleft, 0.0f, 360.0f, 0.0f, 1.0f);
				float uRightTexCoord = RangeMap(angleRight, 0.0f, 360.0f, 0.0f, 1.0f);
				
				float vBottomTexCoord = RangeMap(angleBottom, 270.0f, 360.0f, 1.0f, 0.5f);
				float vTopTexCoord = RangeMap(angleTop, 270.0f, 360.0f, 1.0f, 0.5f);

				AddVertsForQuad3D(vertices, bl, br, tr, tl, color, AABB2(uLeftTexCoord, vBottomTexCoord, uRightTexCoord, vTopTexCoord));
			}
		}
	}
}

void AddVertsForSphere3D(std::vector<Vertex_PCUTBN>& vertices, Vec3 const& center, float radius, Rgba8 const& color, AABB2 const& UVs, int numLatitudeSlices, int numLongitudeSlices)
{
	UNUSED(UVs);

	float yawTheta = 360.0f / (float)numLongitudeSlices;
	float pitchTheta = (360.0f / (float)numLatitudeSlices);

	for (int pitchIndex = 0; pitchIndex < numLongitudeSlices; pitchIndex++)
	{
		if (pitchTheta * (pitchIndex + 1) >= 0.0f && pitchTheta * (pitchIndex + 1) <= 90.0f + pitchTheta)
		{
			for (int yawIndex = 0; yawIndex < numLatitudeSlices; yawIndex++)
			{
				float angleleft = yawTheta * yawIndex;
				float angleRight = yawTheta + angleleft;
				float angleTop = pitchTheta * pitchIndex;
				float angleBottom = pitchTheta + angleTop;

				Vec3 bl = Vec3::MakeFromPolarDegrees(angleBottom, angleleft, radius);
				Vec3 br = Vec3::MakeFromPolarDegrees(angleBottom, angleRight, radius);
				Vec3 tl = Vec3::MakeFromPolarDegrees(angleTop, angleleft, radius);
				Vec3 tr = Vec3::MakeFromPolarDegrees(angleTop, angleRight, radius);

				bl += center;
				br += center;
				tl += center;
				tr += center;

				float uLeftTexCoord = RangeMap(angleleft, 0.0f, 360.0f, 0.0f, 1.0f);
				float uRightTexCoord = RangeMap(angleRight, 0.0f, 360.0f, 0.0f, 1.0f);

				float vBottomTexCoord = RangeMap(angleBottom, 0.0f, 90.0f, 0.5f, 0.0f);
				float vTopTexCoord = RangeMap(angleTop, 0.0f, 90.0f, 0.5f, 0.0f);

				AddVertsForQuad3D(vertices, bl, br, tr, tl, color, AABB2(uLeftTexCoord, vBottomTexCoord, uRightTexCoord, vTopTexCoord));
			}
		}

		if (pitchTheta * (pitchIndex + 1) >= 270.0f && pitchTheta * (pitchIndex + 1) <= 360.0f)
		{
			for (int yawIndex = 0; yawIndex < numLatitudeSlices; yawIndex++)
			{
				float angleleft = yawTheta * yawIndex;
				float angleRight = yawTheta + angleleft;
				float angleTop = pitchTheta * pitchIndex;
				float angleBottom = pitchTheta + angleTop;

				Vec3 bl = Vec3::MakeFromPolarDegrees(angleBottom, angleleft, radius);
				Vec3 br = Vec3::MakeFromPolarDegrees(angleBottom, angleRight, radius);
				Vec3 tl = Vec3::MakeFromPolarDegrees(angleTop, angleleft, radius);
				Vec3 tr = Vec3::MakeFromPolarDegrees(angleTop, angleRight, radius);

				bl += center;
				br += center;
				tl += center;
				tr += center;

				float uLeftTexCoord = RangeMap(angleleft, 0.0f, 360.0f, 0.0f, 1.0f);
				float uRightTexCoord = RangeMap(angleRight, 0.0f, 360.0f, 0.0f, 1.0f);

				float vBottomTexCoord = RangeMap(angleBottom, 270.0f, 360.0f, 1.0f, 0.5f);
				float vTopTexCoord = RangeMap(angleTop, 270.0f, 360.0f, 1.0f, 0.5f);

				AddVertsForQuad3D(vertices, bl, br, tr, tl, color, AABB2(uLeftTexCoord, vBottomTexCoord, uRightTexCoord, vTopTexCoord));
			}
		}
	}
}

void AddVertsForZSphere(std::vector<Vertex_PCUTBN>& verts, std::vector<unsigned int>& indexes, const Vec3& center, float radius, const Rgba8& color, const AABB2& UVs, int numLatitudeSlices, int numLongitudeSlices)
{
	float const sliceAngle = 360.0f / (float)numLongitudeSlices;
	float const stackAngle = 180.0f / (float)numLatitudeSlices;

	float uvCellWidth = (UVs.m_maxs.x - UVs.m_mins.x) /  (float)numLongitudeSlices;
	float uvCellHeight = (UVs.m_maxs.y - UVs.m_mins.y) / (float)numLatitudeSlices;
	std::vector<Vertex_PCUTBN> prevRow;
	std::vector<Vertex_PCUTBN> currentRow;

	for (int yy = 0; yy <= numLatitudeSlices; yy++)
	{
		currentRow.clear();
		float phi = stackAngle * yy - 90.0f;
		float cosPhi = CosDegrees(phi);
		float sinPhi = SinDegrees(phi);

		for (int xx = 0; xx <= numLongitudeSlices; xx++)
		{
			float theta = sliceAngle * xx;
			float cosTheta = CosDegrees(theta);
			float sinTheta = SinDegrees(theta);

			Vertex_PCUTBN currentVert;

			Vec3 spherePos = Vec3(float(cosTheta * cosPhi), float(sinTheta * cosPhi), sinPhi) * radius + center;

			currentVert.m_position = spherePos;
			currentVert.m_color = color;
			currentVert.m_uvTexCoords = Vec2(UVs.m_mins.x + uvCellWidth * xx, UVs.m_mins.y + uvCellHeight * yy);
			currentVert.m_normal = (spherePos - center).GetNormalized();
			currentVert.m_tangent = CrossProduct3D(Vec3(0.0f, 0.0f, 1.0f), currentVert.m_normal).GetNormalized();
			currentVert.m_biTangent = CrossProduct3D(currentVert.m_normal, currentVert.m_tangent).GetNormalized();

			currentRow.push_back(currentVert);
		}

		if (!prevRow.empty())
		{
			for (int i = 0; i < numLongitudeSlices; i++)
			{
				Vertex_PCUTBN bl = prevRow[i];
				Vertex_PCUTBN br = prevRow[i + 1];
				Vertex_PCUTBN tl = currentRow[i];
				Vertex_PCUTBN tr = currentRow[i + 1];

				verts.push_back(br);
				verts.push_back(bl);
				verts.push_back(tl);
				verts.push_back(tr);

				int currentIndex = (int)verts.size() - 4;

				indexes.push_back(currentIndex + 2);
				indexes.push_back(currentIndex + 1);
				indexes.push_back(currentIndex + 0);
				indexes.push_back(currentIndex + 3);
				indexes.push_back(currentIndex + 2);
				indexes.push_back(currentIndex + 0);
			}
		}

		prevRow = currentRow;
	}
}

void AddVertsForWireZSphere3D(std::vector<Vertex_PCU>& vertices, Vec3 const& center, float radius, Rgba8 const& color, AABB2 const& UVs, int numLatitudeSlices, int numLongitudeSlices)
{
	UNUSED(UVs);

	float yawTheta = 360.0f / (float)numLatitudeSlices;
	float pitchTheta = 360.0f / (float)numLongitudeSlices;

	for (int pitchIndex = 0; pitchIndex < numLongitudeSlices; pitchIndex++)
	{
		if (pitchTheta * (pitchIndex + 1) >= pitchTheta && pitchTheta * (pitchIndex + 1) <= 90.0f)
		{
			for (int yawIndex = 0; yawIndex < numLatitudeSlices; yawIndex++)
			{
				float angleleft = yawTheta * yawIndex;
				float angleRight = yawTheta + angleleft;
				float angleTop = pitchTheta * pitchIndex;
				float angleBottom = pitchTheta + angleTop;

				Vec3 bl = Vec3::MakeFromPolarDegrees(angleBottom, angleleft, radius);
				Vec3 br = Vec3::MakeFromPolarDegrees(angleBottom, angleRight, radius);
				Vec3 tl = Vec3::MakeFromPolarDegrees(angleTop, angleleft, radius);
				Vec3 tr = Vec3::MakeFromPolarDegrees(angleTop, angleRight, radius);

				bl += center;
				br += center;
				tl += center;
				tr += center;

				AddVertsForCylinder3D(vertices, bl, br, 0.02f, color, UVs, 3);
				AddVertsForCylinder3D(vertices, br, tr, 0.02f, color, UVs, 3);
				AddVertsForCylinder3D(vertices, tr, tl, 0.02f, color, UVs, 3);
				AddVertsForCylinder3D(vertices, tl, bl, 0.02f, color, UVs, 3);
			}
		}

		if (pitchTheta * (pitchIndex + 1) >= 270.0f && pitchTheta * (pitchIndex + 1) <= 360.0f)
		{
			for (int yawIndex = 0; yawIndex < numLatitudeSlices; yawIndex++)
			{
				float angleleft = yawTheta * yawIndex;
				float angleRight = yawTheta + angleleft;
				float angleTop = pitchTheta * pitchIndex;
				float angleBottom = pitchTheta + angleTop;

				Vec3 bl = Vec3::MakeFromPolarDegrees(angleBottom, angleleft, radius);
				Vec3 br = Vec3::MakeFromPolarDegrees(angleBottom, angleRight, radius);
				Vec3 tl = Vec3::MakeFromPolarDegrees(angleTop, angleleft, radius);
				Vec3 tr = Vec3::MakeFromPolarDegrees(angleTop, angleRight, radius);

				bl += center;
				br += center;
				tl += center;
				tr += center;

				AddVertsForCylinder3D(vertices, bl, br, 0.02f, color, UVs, 3);
				AddVertsForCylinder3D(vertices, br, tr, 0.02f, color, UVs, 3);
				AddVertsForCylinder3D(vertices, tr, tl, 0.02f, color, UVs, 3);
				AddVertsForCylinder3D(vertices, tl, bl, 0.02f, color, UVs, 3);
			}
		}
	}
}

void AddVertsForCylinder3D(std::vector<Vertex_PCU>& vertices, Vec3 const& start, Vec3 const& end, float radius, Rgba8 const& color, AABB2 const& UVs, int numSlices)
{
	UNUSED(UVs);

	std::vector<Vertex_PCU> tempVerts;

	Vec3 upBasis = end - start;
	float theta = 360.0f / (float)numSlices;

	Vec3 leftBasis = CrossProduct3D(upBasis, Vec3(0.0f, 0.0f, 1.0f));
	Vec3 fwdBasis = CrossProduct3D(leftBasis, upBasis);
	
	float upLength = upBasis.GetLength();
	float crossSecLength = upLength + (2.0f * radius);

	if(upBasis.GetNormalized() == Vec3(0.0f, 0.0f, 1.0f))
	{
		leftBasis = Vec3(0.0f, 1.0f, 0.0f);
		fwdBasis = Vec3(1.0f, 0.0f, 0.0f);
	}

	if (upBasis.GetNormalized() == Vec3(0.0f, 0.0f, -1.0f))
	{
		leftBasis = Vec3(0.0f, -1.0f, 0.0f);
		fwdBasis = Vec3(-1.0f, 0.0f, 0.0f);
	}

	Mat44 tranformMatrix;

	tranformMatrix.SetIJKT3D(fwdBasis.GetNormalized(), leftBasis.GetNormalized(), upBasis.GetNormalized(), start);

	for (int index = 0; index < numSlices; index++)
	{
		float angleleft = theta * index;
		float angleRight = theta + angleleft;

		Vec3 bl = Vec3::MakeFromPolarDegrees(0.0f, angleleft, radius);
		Vec3 br = Vec3::MakeFromPolarDegrees(0.0f, angleRight, radius);
		Vec3 tl = Vec3(0.0f, 0.0f, upBasis.GetLength()) + Vec3::MakeFromPolarDegrees(0.0f, angleleft, radius);
		Vec3 tr = Vec3(0.0f, 0.0f, upBasis.GetLength()) + Vec3::MakeFromPolarDegrees(0.0f, angleRight, radius);

		float uLeftTexCoord = RangeMap(angleleft, 0.0f, 360.0f, 0.0f, 1.0f);
		float uRightTexCoord = RangeMap(angleRight, 0.0f, 360.0f, 0.0f, 1.0f);
		float centerUCoord = RangeMap(angleleft + (theta / 2.0f), 0.0f, 360.0f, 0.0f, 1.0f);

		AddVertsForQuad3D(tempVerts, bl, br, tr, tl, color, AABB2(uLeftTexCoord, (radius / crossSecLength), uRightTexCoord, ((radius + upLength) / crossSecLength)));

		tempVerts.push_back(Vertex_PCU(Vec3(0.0f, 0.0f, 0.0f), color, Vec2(centerUCoord, 0.0f)));
		tempVerts.push_back(Vertex_PCU(br, color, Vec2(uRightTexCoord, (radius / crossSecLength))));
		tempVerts.push_back(Vertex_PCU(bl, color, Vec2(uLeftTexCoord, (radius / crossSecLength))));

		tempVerts.push_back(Vertex_PCU(Vec3(0.0f, 0.0f, upBasis.GetLength()), color, Vec2(centerUCoord, 1.0f)));
		tempVerts.push_back(Vertex_PCU(tl, color, Vec2(uLeftTexCoord, ((radius + upLength) / crossSecLength))));
		tempVerts.push_back(Vertex_PCU(tr, color, Vec2(uRightTexCoord, ((radius + upLength) / crossSecLength))));
	}

	TransformVertexArray3D(tempVerts, tranformMatrix);

	vertices.insert(vertices.begin(), tempVerts.begin(), tempVerts.end());
}

void AddVertsForCylinder3D(std::vector<Vertex_PCU>& vertices, Vec3 const& start, Vec3 const& end, float radius, Rgba8 const& startColor, Rgba8 const& endColor, AABB2 const& UVs, int numSlices)
{
	UNUSED(UVs);

	std::vector<Vertex_PCU> tempVerts;

	Vec3 upBasis = end - start;
	float theta = 360.0f / (float)numSlices;

	Vec3 leftBasis = CrossProduct3D(upBasis, Vec3(0.0f, 0.0f, 1.0f));
	Vec3 fwdBasis = CrossProduct3D(leftBasis, upBasis);

	float upLength = upBasis.GetLength();
	float crossSecLength = upLength + (2.0f * radius);

	if (upBasis.GetNormalized() == Vec3(0.0f, 0.0f, 1.0f))
	{
		leftBasis = Vec3(0.0f, 1.0f, 0.0f);
		fwdBasis = Vec3(1.0f, 0.0f, 0.0f);
	}

	if (upBasis.GetNormalized() == Vec3(0.0f, 0.0f, -1.0f))
	{
		leftBasis = Vec3(0.0f, -1.0f, 0.0f);
		fwdBasis = Vec3(-1.0f, 0.0f, 0.0f);
	}

	Mat44 tranformMatrix;

	tranformMatrix.SetIJKT3D(fwdBasis.GetNormalized(), leftBasis.GetNormalized(), upBasis.GetNormalized(), start);

	for (int index = 0; index < numSlices; index++)
	{
		float angleleft = theta * index;
		float angleRight = theta + angleleft;

		Vec3 bl = Vec3::MakeFromPolarDegrees(0.0f, angleleft, radius);
		Vec3 br = Vec3::MakeFromPolarDegrees(0.0f, angleRight, radius);
		Vec3 tl = Vec3(0.0f, 0.0f, upBasis.GetLength()) + Vec3::MakeFromPolarDegrees(0.0f, angleleft, radius);
		Vec3 tr = Vec3(0.0f, 0.0f, upBasis.GetLength()) + Vec3::MakeFromPolarDegrees(0.0f, angleRight, radius);

		float uLeftTexCoord = RangeMap(angleleft, 0.0f, 360.0f, 0.0f, 1.0f);
		float uRightTexCoord = RangeMap(angleRight, 0.0f, 360.0f, 0.0f, 1.0f);
		float centerUCoord = RangeMap(angleleft + (theta / 2.0f), 0.0f, 360.0f, 0.0f, 1.0f);

		AddVertsForQuad3D(tempVerts, bl, br, tr, tl, startColor, endColor, AABB2(uLeftTexCoord, (radius / crossSecLength), uRightTexCoord, ((radius + upLength) / crossSecLength)));

		tempVerts.push_back(Vertex_PCU(Vec3(0.0f, 0.0f, 0.0f), startColor, Vec2(centerUCoord, 0.0f)));
		tempVerts.push_back(Vertex_PCU(br, startColor, Vec2(uRightTexCoord, (radius / crossSecLength))));
		tempVerts.push_back(Vertex_PCU(bl, startColor, Vec2(uLeftTexCoord, (radius / crossSecLength))));

		tempVerts.push_back(Vertex_PCU(Vec3(0.0f, 0.0f, upBasis.GetLength()), endColor, Vec2(centerUCoord, 1.0f)));
		tempVerts.push_back(Vertex_PCU(tl, endColor, Vec2(uLeftTexCoord, ((radius + upLength) / crossSecLength))));
		tempVerts.push_back(Vertex_PCU(tr, endColor, Vec2(uRightTexCoord, ((radius + upLength) / crossSecLength))));
	}

	TransformVertexArray3D(tempVerts, tranformMatrix);

	vertices.insert(vertices.begin(), tempVerts.begin(), tempVerts.end());
}

void AddVertsForZCylinder3D(std::vector<Vertex_PCU>& vertices, Vec3 const& start, float const& height, float radius, Rgba8 const& color, AABB2 const& UVs, int numSlices)
{
	UNUSED(UVs);

	std::vector<Vertex_PCU> tempVerts;

	float theta = 360.0f / (float)numSlices;

	Vec3 upBasis = Vec3(0.0f, 0.0f, 1.0f);
	Vec3 leftBasis = Vec3(0.0f, 1.0f, 0.0f);
	Vec3 fwdBasis = Vec3(1.0f, 0.0f, 0.0f);

	float upLength = upBasis.GetLength();
	float crossSecLength = upLength + (2.0f * radius);

	Mat44 tranformMatrix;

	tranformMatrix.SetIJKT3D(fwdBasis.GetNormalized(), leftBasis.GetNormalized(), upBasis.GetNormalized(), start);

	for (int index = 0; index < numSlices; index++)
	{
		float angleleft = theta * index;
		float angleRight = theta + angleleft;

		Vec3 bl = Vec3::MakeFromPolarDegrees(0.0f, angleleft, radius);
		Vec3 br = Vec3::MakeFromPolarDegrees(0.0f, angleRight, radius);
		Vec3 tl = Vec3(0.0f, 0.0f, height) + Vec3::MakeFromPolarDegrees(0.0f, angleleft, radius);
		Vec3 tr = Vec3(0.0f, 0.0f, height) + Vec3::MakeFromPolarDegrees(0.0f, angleRight, radius);

		float uLeftTexCoord = RangeMap(angleleft, 0.0f, 360.0f, 0.0f, 1.0f);
		float uRightTexCoord = RangeMap(angleRight, 0.0f, 360.0f, 0.0f, 1.0f);
		float centerUCoord = RangeMap(angleleft + (theta / 2.0f), 0.0f, 360.0f, 0.0f, 1.0f);

		AddVertsForQuad3D(tempVerts, bl, br, tr, tl, color, AABB2(uLeftTexCoord, 0.0f, uRightTexCoord, 1.0f));

		tempVerts.push_back(Vertex_PCU(Vec3(0.0f, 0.0f, 0.0f), color, Vec2(centerUCoord, 0.0f)));
		tempVerts.push_back(Vertex_PCU(br, color, Vec2(uRightTexCoord, (radius / crossSecLength))));
		tempVerts.push_back(Vertex_PCU(bl, color, Vec2(uLeftTexCoord, (radius / crossSecLength))));

		tempVerts.push_back(Vertex_PCU(Vec3(0.0f, 0.0f, height), color, Vec2(centerUCoord, 1.0f)));
		tempVerts.push_back(Vertex_PCU(tl, color, Vec2(uLeftTexCoord, ((radius + upLength) / crossSecLength))));
		tempVerts.push_back(Vertex_PCU(tr, color, Vec2(uRightTexCoord, ((radius + upLength) / crossSecLength))));
	}

	TransformVertexArray3D(tempVerts, tranformMatrix);

	vertices.insert(vertices.begin(), tempVerts.begin(), tempVerts.end());
}

void AddVertsForZWireCylinder3D(std::vector<Vertex_PCU>& vertices, Vec3 const& start, float const& height, float radius, Rgba8 const& color, AABB2 const& UVs, int numSlices)
{
	UNUSED(UVs);

	std::vector<Vertex_PCU> tempVerts;

	float theta = 360.0f / (float)numSlices;

	Vec3 upBasis = Vec3(0.0f, 0.0f, 1.0f);
	Vec3 leftBasis = Vec3(0.0f, 1.0f, 0.0f);
	Vec3 fwdBasis = Vec3(1.0f, 0.0f, 0.0f);

	Mat44 tranformMatrix;

	tranformMatrix.SetIJKT3D(fwdBasis.GetNormalized(), leftBasis.GetNormalized(), upBasis.GetNormalized(), start);

	for (int index = 0; index < numSlices; index++)
	{
		float angleleft = theta * index;
		float angleRight = theta + angleleft;

		Vec3 bl = start + Vec3::MakeFromPolarDegrees(0.0f, angleleft, radius);
		Vec3 br = start + Vec3::MakeFromPolarDegrees(0.0f, angleRight, radius);
		Vec3 tl = start + Vec3(0.0f, 0.0f, height) + Vec3::MakeFromPolarDegrees(0.0f, angleleft, radius);
		Vec3 tr = start + Vec3(0.0f, 0.0f, height) + Vec3::MakeFromPolarDegrees(0.0f, angleRight, radius);

		AddVertsForCylinder3D(vertices, bl, br, 0.02f, color, UVs, 3);
		AddVertsForCylinder3D(vertices, br, tr, 0.02f, color, UVs, 3);
		AddVertsForCylinder3D(vertices, tr, tl, 0.02f, color, UVs, 3);
		AddVertsForCylinder3D(vertices, tl, bl, 0.02f, color, UVs, 3);

		AddVertsForCylinder3D(vertices, start, br, 0.02f, color, UVs, 3);
		AddVertsForCylinder3D(vertices, start, bl, 0.02f, color, UVs, 3);

		AddVertsForCylinder3D(vertices, start + Vec3(0.0f, 0.0f, height), tr, 0.02f, color, UVs, 3);
		AddVertsForCylinder3D(vertices, start + Vec3(0.0f, 0.0f, height), tl, 0.02f, color, UVs, 3);

	}
}

void AddVertsForCone3D(std::vector<Vertex_PCU>& vertices, Vec3 const& start, Vec3 const& end, float radius, Rgba8 const& color, AABB2 const& UVs, int numSlices)
{
	std::vector<Vertex_PCU> tempVerts;

	Vec3 upBasis = end - start;
	float theta = 360.0f / (float)numSlices;

	Vec3 leftBasis = CrossProduct3D(upBasis, Vec3(0.0f, 0.0f, 1.0f));
	Vec3 fwdBasis = CrossProduct3D(leftBasis, upBasis);

	if (upBasis.GetNormalized() == Vec3(0.0f, 0.0f, 1.0f))
	{
		leftBasis = Vec3(0.0f, 1.0f, 0.0f);
		fwdBasis = Vec3(1.0f, 0.0f, 0.0f);
	}

	if (upBasis.GetNormalized() == Vec3(0.0f, 0.0f, -1.0f))
	{
		leftBasis = Vec3(0.0f, -1.0f, 0.0f);
		fwdBasis = Vec3(-1.0f, 0.0f, 0.0f);
	}

	Mat44 tranformMatrix;

	tranformMatrix.SetIJKT3D(fwdBasis.GetNormalized(), leftBasis.GetNormalized(), upBasis.GetNormalized(), start);

	for (int index = 0; index < numSlices; index++)
	{
		float angleleft = theta * index;
		float angleRight = theta + angleleft;

		Vec3 bl = Vec3::MakeFromPolarDegrees(0.0f, angleleft, radius);
		Vec3 br = Vec3::MakeFromPolarDegrees(0.0f, angleRight, radius);

		tempVerts.push_back(Vertex_PCU(Vec3(0.0f, 0.0f, 0.0f), color, UVs.m_mins));
		tempVerts.push_back(Vertex_PCU(br, color, UVs.m_mins));
		tempVerts.push_back(Vertex_PCU(bl, color, UVs.m_mins));

		tempVerts.push_back(Vertex_PCU(Vec3(0.0f, 0.0f, upBasis.GetLength()), color, UVs.m_mins));
		tempVerts.push_back(Vertex_PCU(bl, color, UVs.m_mins));
		tempVerts.push_back(Vertex_PCU(br, color, UVs.m_mins));
	}

	TransformVertexArray3D(tempVerts, tranformMatrix);

	vertices.insert(vertices.begin(), tempVerts.begin(), tempVerts.end());
}

void AddVertsForArrow(std::vector<Vertex_PCU>& vertices, Vec3 const& start, Vec3 const& end, float radius, Rgba8 const& color, AABB2 const& UVs)
{
	Vec3 cylinderEnd = start + (0.7f * (end - start));

	AddVertsForCylinder3D(vertices, start, cylinderEnd, radius, color, UVs, 4);
	AddVertsForCone3D(vertices, cylinderEnd, end, radius + (radius * 0.5f), color, UVs, 4);
}

void AddVertsForHollowHexagon3D(std::vector<Vertex_PCU>& vertices, Vec3 const& center, float width, float thickness, Rgba8 const& color)
{
	float sideLength = (width * 0.5f) * 1.1547f;

	for (int i = 0; i < 6; i++)
	{
		float theta1 = 60.0f * i;
		float theta2 = 60.0f * (i + 1);

		Vec3 bl;
		Vec3 br;
		Vec3 tr;
		Vec3 tl;

		bl.x = center.x + (sideLength - (thickness * 0.5f)) * CosDegrees(theta1);
		bl.y = center.y + (sideLength - (thickness * 0.5f)) * SinDegrees(theta1);
		bl.z = center.z;

		br.x = center.x + (sideLength + (thickness * 0.5f)) * CosDegrees(theta1);
		br.y = center.y + (sideLength + (thickness * 0.5f)) * SinDegrees(theta1);
		br.z = center.z;

		tr.x = center.x + (sideLength + (thickness * 0.5f)) * CosDegrees(theta2);
		tr.y = center.y + (sideLength + (thickness * 0.5f)) * SinDegrees(theta2);
		tr.z = center.z;
	   
		tl.x = center.x + (sideLength - (thickness * 0.5f)) * CosDegrees(theta2);
		tl.y = center.y + (sideLength - (thickness * 0.5f)) * SinDegrees(theta2);
		tl.z = center.z;

		vertices.push_back(Vertex_PCU(bl, color, Vec2::ZERO));
		vertices.push_back(Vertex_PCU(br, color, Vec2::ZERO));
		vertices.push_back(Vertex_PCU(tr, color, Vec2::ZERO));

		vertices.push_back(Vertex_PCU(tr, color, Vec2::ZERO));
		vertices.push_back(Vertex_PCU(tl, color, Vec2::ZERO));
		vertices.push_back(Vertex_PCU(bl, color, Vec2::ZERO));
	}
}

void AddVertsForHexagon3D(std::vector<Vertex_PCU>& vertices, Vec3 const& center, float width, float thickness, Rgba8 const& color)
{
	float sideLength = (width * 0.5f) * 1.1547f;

	for (int i = 0; i < 6; i++)
	{
		float theta1 = 60.0f * i;
		float theta2 = 60.0f * (i + 1);

		Vec3 v1;
		Vec3 v2;

		v1.x = center.x + (sideLength - (thickness * 0.5f)) * CosDegrees(theta1);
		v1.y = center.y + (sideLength - (thickness * 0.5f)) * SinDegrees(theta1);
		v1.z = center.z;

		v2.x = center.x + (sideLength - (thickness * 0.5f)) * CosDegrees(theta2);
		v2.y = center.y + (sideLength - (thickness * 0.5f)) * SinDegrees(theta2);
		v2.z = center.z;

		vertices.push_back(Vertex_PCU(center, color, Vec2::ZERO));
		vertices.push_back(Vertex_PCU(v1, color, Vec2::ZERO));
		vertices.push_back(Vertex_PCU(v2, color, Vec2::ZERO));
	}
}

void CalculateTangentSpaceBasisVectors(std::vector<Vertex_PCUTBN>& vertices, std::vector<unsigned int> indices, bool computeNormals, bool computeTangents)
{
	for (int i = 0; i < (int)indices.size(); i += 3)
	{
		unsigned int triIndices[3] = 
		{
			indices[i + 0],
			indices[i + 1],
			indices[i + 2]
		};

		Vertex_PCUTBN triVerts[3] = 
		{
			vertices[triIndices[0]],
			vertices[triIndices[1]],
			vertices[triIndices[2]]
		};

		Vec3 edge1 = triVerts[1].m_position - triVerts[0].m_position;
		Vec3 edge2 = triVerts[2].m_position - triVerts[0].m_position;

		if (computeNormals)
		{
			vertices[triIndices[0]].m_normal = Vec3();
			vertices[triIndices[1]].m_normal = Vec3();
			vertices[triIndices[2]].m_normal = Vec3();

			Vec3 triangleNormal = CrossProduct3D(edge1.GetNormalized(), edge2.GetNormalized()).GetNormalized();

			vertices[triIndices[0]].m_normal += triangleNormal;
			vertices[triIndices[1]].m_normal += triangleNormal;
			vertices[triIndices[2]].m_normal += triangleNormal;
		}

		if (computeTangents)
		{
			vertices[triIndices[0]].m_tangent = Vec3();
			vertices[triIndices[1]].m_tangent = Vec3();
			vertices[triIndices[2]].m_tangent = Vec3();

			vertices[triIndices[0]].m_biTangent = Vec3();
			vertices[triIndices[1]].m_biTangent = Vec3();
			vertices[triIndices[2]].m_biTangent = Vec3();

			float deltaU0 = triVerts[1].m_uvTexCoords.x - triVerts[0].m_uvTexCoords.x;
			float deltaU1 = triVerts[2].m_uvTexCoords.x - triVerts[0].m_uvTexCoords.x;

			float deltaV0 = triVerts[1].m_uvTexCoords.y - triVerts[0].m_uvTexCoords.y;
			float deltaV1 = triVerts[2].m_uvTexCoords.y - triVerts[0].m_uvTexCoords.y;

			float r = 1 / ((deltaU0 * deltaV1) - (deltaU1 * deltaV0));

			Vec3 tangent = r * ((deltaV1 * edge1) - (deltaV0 * edge2));
			Vec3 biTangent = r * ((deltaU0 * edge2) - (deltaU1 * edge1));

			vertices[triIndices[0]].m_tangent += tangent.GetNormalized();
			vertices[triIndices[1]].m_tangent += tangent.GetNormalized();
			vertices[triIndices[2]].m_tangent += tangent.GetNormalized();

			vertices[triIndices[0]].m_biTangent += biTangent.GetNormalized();
			vertices[triIndices[1]].m_biTangent += biTangent.GetNormalized();
			vertices[triIndices[2]].m_biTangent += biTangent.GetNormalized();
		}
	}

	for (int i = 0; i < (int)vertices.size(); i++)
	{
		Vec3 vertNormal = vertices[i].m_normal;
		Vec3 vertTangent = vertices[i].m_tangent;
		Vec3 vertBiTangent = vertices[i].m_biTangent;

		Vec3 correctedTangent = CrossProduct3D(vertBiTangent.GetNormalized(), vertNormal.GetNormalized()).GetNormalized();
		Vec3 correctBiTangent = CrossProduct3D(vertNormal.GetNormalized(), correctedTangent.GetNormalized()).GetNormalized();

		vertices[i].m_normal = vertNormal;
		vertices[i].m_tangent = correctedTangent;
		vertices[i].m_biTangent = correctBiTangent;
	}
}

void CalculateTangentSpaceBasisVectors(std::vector<MeshVertex_PCUTBN>& vertices, std::vector<unsigned int> indices, bool computeNormals, bool computeTangents)
{
	for (int i = 0; i < (int)indices.size(); i += 3)
	{
		unsigned int triIndices[3] = 
		{
			indices[i + 0],
			indices[i + 1],
			indices[i + 2]
		};

		MeshVertex_PCUTBN triVerts[3] = 
		{
			vertices[triIndices[0]],
			vertices[triIndices[1]],
			vertices[triIndices[2]]
		};

		Vec3 edge1 = triVerts[1].m_position - triVerts[0].m_position;
		Vec3 edge2 = triVerts[2].m_position - triVerts[0].m_position;

		if (computeNormals)
		{
			vertices[triIndices[0]].m_normal = Vec3();
			vertices[triIndices[1]].m_normal = Vec3();
			vertices[triIndices[2]].m_normal = Vec3();

			Vec3 triangleNormal = CrossProduct3D(edge1.GetNormalized(), edge2.GetNormalized()).GetNormalized();

			vertices[triIndices[0]].m_normal += triangleNormal;
			vertices[triIndices[1]].m_normal += triangleNormal;
			vertices[triIndices[2]].m_normal += triangleNormal;
		}

		if (computeTangents)
		{
			vertices[triIndices[0]].m_tangent = Vec3();
			vertices[triIndices[1]].m_tangent = Vec3();
			vertices[triIndices[2]].m_tangent = Vec3();

			vertices[triIndices[0]].m_biTangent = Vec3();
			vertices[triIndices[1]].m_biTangent = Vec3();
			vertices[triIndices[2]].m_biTangent = Vec3();

			float deltaU0 = triVerts[1].m_uv.x - triVerts[0].m_uv.x;
			float deltaU1 = triVerts[2].m_uv.x - triVerts[0].m_uv.x;

			float deltaV0 = triVerts[1].m_uv.y - triVerts[0].m_uv.y;
			float deltaV1 = triVerts[2].m_uv.y - triVerts[0].m_uv.y;

			float r = 1 / ((deltaU0 * deltaV1) - (deltaU1 * deltaV0));

			Vec3 tangent = r * ((deltaV1 * edge1) - (deltaV0 * edge2));
			Vec3 biTangent = r * ((deltaU0 * edge2) - (deltaU1 * edge1));

			vertices[triIndices[0]].m_tangent += tangent.GetNormalized();
			vertices[triIndices[1]].m_tangent += tangent.GetNormalized();
			vertices[triIndices[2]].m_tangent += tangent.GetNormalized();

			vertices[triIndices[0]].m_biTangent += biTangent.GetNormalized();
			vertices[triIndices[1]].m_biTangent += biTangent.GetNormalized();
			vertices[triIndices[2]].m_biTangent += biTangent.GetNormalized();
		}
	}

	for (int i = 0; i < (int)vertices.size(); i++)
	{
		Vec3 vertNormal = vertices[i].m_normal;
		Vec3 vertTangent = vertices[i].m_tangent;
		Vec3 vertBiTangent = vertices[i].m_biTangent;

		Vec3 correctedTangent = CrossProduct3D(vertBiTangent.GetNormalized(), vertNormal.GetNormalized()).GetNormalized();
		Vec3 correctBiTangent = CrossProduct3D(vertNormal.GetNormalized(), correctedTangent.GetNormalized()).GetNormalized();

		vertices[i].m_normal = vertNormal;
		vertices[i].m_tangent = correctedTangent;
		vertices[i].m_biTangent = correctBiTangent;
	}
}

AABB2 GetVertexBounds2D(std::vector<Vertex_PCU> const& verts)
{
	float vertexMinsX = verts[0].m_position.x;
	float vertexMaxsX = verts[0].m_position.x;
	float vertexMinsY = verts[0].m_position.y;
	float vertexMaxsY = verts[0].m_position.y;

	for (size_t index = 0; index < verts.size(); index++)
	{
		if (vertexMinsX > verts[index].m_position.x)
			vertexMinsX = verts[index].m_position.x;

		if (vertexMaxsX < verts[index].m_position.x)
			vertexMaxsX = verts[index].m_position.x;

		if (vertexMinsY > verts[index].m_position.y)
			vertexMinsY = verts[index].m_position.y;

		if (vertexMaxsY < verts[index].m_position.y)
			vertexMaxsY = verts[index].m_position.y;
	}

	return AABB2(vertexMinsX, vertexMinsY, vertexMaxsX, vertexMaxsY);
}
