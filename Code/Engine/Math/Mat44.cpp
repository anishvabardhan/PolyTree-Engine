#include "Engine/Math/Mat44.hpp"

#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/Vec4.hpp"
#include "Engine/Math/MathUtils.hpp"

Mat44::Mat44()
{
	m_values[Ix] = 1;
	m_values[Iy] = 0;
	m_values[Iz] = 0;
	m_values[Iw] = 0;
	m_values[Jx] = 0;
	m_values[Jy] = 1;
	m_values[Jz] = 0;
	m_values[Jw] = 0;
	m_values[Kx] = 0;
	m_values[Ky] = 0;
	m_values[Kz] = 1;
	m_values[Kw] = 0;
	m_values[Tx] = 0;
	m_values[Ty] = 0;
	m_values[Tz] = 0;
	m_values[Tw] = 1;
}

Mat44::Mat44(Vec2 const& iBasis2D, Vec2 const& jBasis2D, Vec2 const& translation2D)
{
	m_values[Ix] = iBasis2D.x;
	m_values[Iy] = iBasis2D.y;
	m_values[Iz] = 0;
	m_values[Iw] = 0;
	m_values[Jx] = jBasis2D.x;
	m_values[Jy] = jBasis2D.y;
	m_values[Jz] = 0;
	m_values[Jw] = 0;
	m_values[Kx] = 0;
	m_values[Ky] = 0;
	m_values[Kz] = 1;
	m_values[Kw] = 0;
	m_values[Tx] = translation2D.x;
	m_values[Ty] = translation2D.y;
	m_values[Tz] = 0;
	m_values[Tw] = 1;
}

Mat44::Mat44(Vec3 const& iBasis3D, Vec3 const& jBasis3D, Vec3 const& kBasis3D, Vec3 const& translation3D)
{
	m_values[Ix] = iBasis3D.x;
	m_values[Iy] = iBasis3D.y;
	m_values[Iz] = iBasis3D.z;
	m_values[Iw] = 0;
	m_values[Jx] = jBasis3D.x;
	m_values[Jy] = jBasis3D.y;
	m_values[Jz] = jBasis3D.z;
	m_values[Jw] = 0;
	m_values[Kx] = kBasis3D.x;
	m_values[Ky] = kBasis3D.y;
	m_values[Kz] = kBasis3D.z;
	m_values[Kw] = 0;
	m_values[Tx] = translation3D.x;
	m_values[Ty] = translation3D.y;
	m_values[Tz] = translation3D.z;
	m_values[Tw] = 1;
}

Mat44::Mat44(Vec4 const& iBasis4D, Vec4 const& jBasis4D, Vec4 const& kBasis4D, Vec4 const& translation4D)
{
	m_values[Ix] = iBasis4D.x;
	m_values[Iy] = iBasis4D.y;
	m_values[Iz] = iBasis4D.z;
	m_values[Iw] = iBasis4D.w;
	m_values[Jx] = jBasis4D.x;
	m_values[Jy] = jBasis4D.y;
	m_values[Jz] = jBasis4D.z;
	m_values[Jw] = jBasis4D.w;
	m_values[Kx] = kBasis4D.x;
	m_values[Ky] = kBasis4D.y;
	m_values[Kz] = kBasis4D.z;
	m_values[Kw] = kBasis4D.w;
	m_values[Tx] = translation4D.x;
	m_values[Ty] = translation4D.y;
	m_values[Tz] = translation4D.z;
	m_values[Tw] = translation4D.w;
}

Mat44::Mat44(float const* sizteenValuesBasisMajor)
{
	m_values[Ix] = sizteenValuesBasisMajor[Ix];
	m_values[Iy] = sizteenValuesBasisMajor[Iy];
	m_values[Iz] = sizteenValuesBasisMajor[Iz];
	m_values[Iw] = sizteenValuesBasisMajor[Iw];
	m_values[Jx] = sizteenValuesBasisMajor[Jx];
	m_values[Jy] = sizteenValuesBasisMajor[Jy];
	m_values[Jz] = sizteenValuesBasisMajor[Jz];
	m_values[Jw] = sizteenValuesBasisMajor[Jw];
	m_values[Kx] = sizteenValuesBasisMajor[Kx];
	m_values[Ky] = sizteenValuesBasisMajor[Ky];
	m_values[Kz] = sizteenValuesBasisMajor[Kz];
	m_values[Kw] = sizteenValuesBasisMajor[Kw];
	m_values[Tx] = sizteenValuesBasisMajor[Tx];
	m_values[Ty] = sizteenValuesBasisMajor[Ty];
	m_values[Tz] = sizteenValuesBasisMajor[Tz];
	m_values[Tw] = sizteenValuesBasisMajor[Tw];
}

Mat44 const Mat44::CreateTranslation2D(Vec2 const& translationXY)
{
	Mat44 matrix;

	matrix.m_values[Tx] = translationXY.x;
	matrix.m_values[Ty] = translationXY.y;

	return matrix;
}

Mat44 const Mat44::CreateTranslation3D(Vec3 const& translationXYZ)
{
	Mat44 matrix;

	matrix.m_values[Tx] = translationXYZ.x;
	matrix.m_values[Ty] = translationXYZ.y;
	matrix.m_values[Tz] = translationXYZ.z;

	return matrix;
}

Mat44 const Mat44::CreateUniformScale2D(float uniformScaleXY)
{
	Mat44 matrix;

	matrix.m_values[Ix] = uniformScaleXY;
	matrix.m_values[Jy] = uniformScaleXY;

	return matrix;
}

Mat44 const Mat44::CreateUniformScale3D(float uniformScaleXYZ)
{
	Mat44 matrix;

	matrix.m_values[Ix] = uniformScaleXYZ;
	matrix.m_values[Jy] = uniformScaleXYZ;
	matrix.m_values[Kz] = uniformScaleXYZ;

	return matrix;
}

Mat44 const Mat44::CreateNonUniformScale2D(Vec2 const& nonUniformScaleXY)
{
	Mat44 matrix;

	matrix.m_values[Ix] = nonUniformScaleXY.x;
	matrix.m_values[Jy] = nonUniformScaleXY.y;

	return matrix;
}

Mat44 const Mat44::CreateNonUniformScale3D(Vec3 const& nonUniformScaleXYZ)
{
	Mat44 matrix;

	matrix.m_values[Ix] = nonUniformScaleXYZ.x;
	matrix.m_values[Jy] = nonUniformScaleXYZ.y;
	matrix.m_values[Kz] = nonUniformScaleXYZ.z;

	return matrix;
}

Mat44 const Mat44::CreateZRotationDegrees(float rotationDegreesAboutZ)
{
	Mat44 matrix;

	matrix.m_values[Ix] = CosDegrees(rotationDegreesAboutZ);
	matrix.m_values[Iy] = SinDegrees(rotationDegreesAboutZ);
	matrix.m_values[Jx] = -1.0f * SinDegrees(rotationDegreesAboutZ);
	matrix.m_values[Jy] = CosDegrees(rotationDegreesAboutZ);

	return matrix;
}

Mat44 const Mat44::CreateYRotationDegrees(float rotationDegreesAboutY)
{
	Mat44 matrix;

	matrix.m_values[Ix] = CosDegrees(rotationDegreesAboutY);
	matrix.m_values[Iz] = -1.0f * SinDegrees(rotationDegreesAboutY);
	matrix.m_values[Kx] = SinDegrees(rotationDegreesAboutY);
	matrix.m_values[Kz] = CosDegrees(rotationDegreesAboutY);

	return matrix;
}

Mat44 const Mat44::CreateXRotationDegrees(float rotationDegreesAboutX)
{
	Mat44 matrix;

	matrix.m_values[Jy] = CosDegrees(rotationDegreesAboutX);
	matrix.m_values[Jz] = SinDegrees(rotationDegreesAboutX);
	matrix.m_values[Ky] = -1.0f * SinDegrees(rotationDegreesAboutX);
	matrix.m_values[Kz] = CosDegrees(rotationDegreesAboutX);

	return matrix;
}

Mat44 const Mat44::CreateOrthoProjection(float minX, float maxX, float minY, float maxY, float minZ, float maxZ)
{
	Mat44 orthoMatrix;

	orthoMatrix.m_values[Ix] = 2.0f / (maxX - minX);
	orthoMatrix.m_values[Jy] = 2.0f / (maxY - minY);
	orthoMatrix.m_values[Kz] = 1.0f / (maxZ - minZ);

	orthoMatrix.m_values[Tx] = (maxX + minX) / (minX - maxX);
	orthoMatrix.m_values[Ty] = (maxY + minY) / (minY - maxY);
	orthoMatrix.m_values[Tz] = minZ / (minZ - maxZ);

	return orthoMatrix;
}

Mat44 const Mat44::CreatePerspectiveProjection(float fov, float aspectRatio, float frontZ, float backZ)
{
	float height = 1.0f / TanDegrees(fov * 0.5f);
	float zRange = backZ - frontZ;
	float q = 1.0f / zRange;

	Mat44 result;

	result.m_values[Ix] = height / aspectRatio;
	result.m_values[Jy] = height;
	result.m_values[Kz] = backZ * q;
	result.m_values[Tz] = -frontZ * backZ * q;
	result.m_values[Kw] = 1.0f;
	result.m_values[Tw] = 0.0f;

	return result;
}

Vec2 const Mat44::TransformVectorQuantity2D(Vec2 const& vectorQuantityXY) const
{
	Vec4 aX = Vec4(m_values[Ix], m_values[Jx], m_values[Kx], m_values[Tx]);
	Vec4 aY = Vec4(m_values[Iy], m_values[Jy], m_values[Ky], m_values[Ty]);

	Vec4 vectorQuantity = Vec4(vectorQuantityXY.x, vectorQuantityXY.y, 0.0f, 0.0f);

	return Vec2(DotProduct2D(aX, vectorQuantity), DotProduct2D(aY, vectorQuantity));
}

Vec3 const Mat44::TransformVectorQuantity3D(Vec3 const& vectorQuantityXYZ) const
{
	Vec4 aX = Vec4(m_values[Ix], m_values[Jx], m_values[Kx], m_values[Tx]);
	Vec4 aY = Vec4(m_values[Iy], m_values[Jy], m_values[Ky], m_values[Ty]);
	Vec4 aZ = Vec4(m_values[Iz], m_values[Jz], m_values[Kz], m_values[Tz]);

	Vec4 vectorQuantity = Vec4(vectorQuantityXYZ.x, vectorQuantityXYZ.y, vectorQuantityXYZ.z, 0.0f);

	return Vec3(DotProduct2D(aX, vectorQuantity), DotProduct2D(aY, vectorQuantity), DotProduct2D(aZ, vectorQuantity));
}

Vec2 const Mat44::TransformPosition2D(Vec2 const& positionXY) const
{
	Vec4 aX = Vec4(m_values[Ix], m_values[Jx], m_values[Kx], m_values[Tx]);
	Vec4 aY = Vec4(m_values[Iy], m_values[Jy], m_values[Ky], m_values[Ty]);

	Vec4 vectorQuantity = Vec4(positionXY.x, positionXY.y, 0.0f, 1.0f);

	return Vec2(DotProduct2D(aX, vectorQuantity), DotProduct2D(aY, vectorQuantity));
}

Vec3 const Mat44::TransformPosition3D(Vec3 const& positionXYZ) const
{
	Vec4 aX = Vec4(m_values[Ix], m_values[Jx], m_values[Kx], m_values[Tx]);
	Vec4 aY = Vec4(m_values[Iy], m_values[Jy], m_values[Ky], m_values[Ty]);
	Vec4 aZ = Vec4(m_values[Iz], m_values[Jz], m_values[Kz], m_values[Tz]);

	Vec4 vectorQuantity = Vec4(positionXYZ.x, positionXYZ.y, positionXYZ.z, 1.0f);

	return Vec3(DotProduct2D(aX, vectorQuantity), DotProduct2D(aY, vectorQuantity), DotProduct2D(aZ, vectorQuantity));
}

Vec4 const Mat44::TransformHomogeneous3D(Vec4 const& homogeneousPoint3D) const
{
	Vec4 aX = Vec4(m_values[Ix], m_values[Jx], m_values[Kx], m_values[Tx]);
	Vec4 aY = Vec4(m_values[Iy], m_values[Jy], m_values[Ky], m_values[Ty]);
	Vec4 aZ = Vec4(m_values[Iz], m_values[Jz], m_values[Kz], m_values[Tz]);
	Vec4 aW = Vec4(m_values[Iw], m_values[Jw], m_values[Kw], m_values[Tw]);

	return Vec4(DotProduct2D(aX, homogeneousPoint3D), DotProduct2D(aY, homogeneousPoint3D), DotProduct2D(aZ, homogeneousPoint3D), DotProduct2D(aW, homogeneousPoint3D));
}

float* Mat44::GetAsFloatArray()
{
	return m_values;
}

float const* Mat44::GetAsFloatArray() const
{
	return m_values;
}

Vec2 const Mat44::GetIBasis2D() const
{
	return Vec2(m_values[Ix], m_values[Iy]);
}

Vec2 const Mat44::GetJBasis2D() const
{
	return Vec2(m_values[Jx], m_values[Jy]);
}

Vec2 const Mat44::GetTranslation2D() const
{
	return Vec2(m_values[Tx], m_values[Ty]);
}

Vec3 const Mat44::GetIBasis3D() const
{
	return Vec3(m_values[Ix], m_values[Iy], m_values[Iz]);
}

Vec3 const Mat44::GetJBasis3D() const
{
	return Vec3(m_values[Jx], m_values[Jy], m_values[Jz]);
}

Vec3 const Mat44::GetKBasis3D() const
{
	return Vec3(m_values[Kx], m_values[Ky], m_values[Kz]);
}

Vec3 const Mat44::GetTranslation3D() const
{
	return Vec3(m_values[Tx], m_values[Ty], m_values[Tz]);
}

Vec4 const Mat44::GetIBasis4D() const
{
	return Vec4(m_values[Ix], m_values[Iy], m_values[Iz], m_values[Iw]);
}

Vec4 const Mat44::GetJBasis4D() const
{
	return Vec4(m_values[Jx], m_values[Jy], m_values[Jz], m_values[Jw]);
}

Vec4 const Mat44::GetKBasis4D() const
{
	return Vec4(m_values[Kx], m_values[Ky], m_values[Kz], m_values[Kw]);
}

Vec4 const Mat44::GetTranslation4D() const
{
	return Vec4(m_values[Tx], m_values[Ty], m_values[Tz], m_values[Tw]);
}

Mat44 const Mat44::GetOrthonormalInverse() const
{
	Mat44 rotation = Mat44(GetIBasis3D(), GetJBasis3D(), GetKBasis3D(), Vec3(0.0f, 0.0f, 0.0f));
	Mat44 translationInverse;

	translationInverse.SetTranslation3D(Vec3(-GetTranslation3D().x, -GetTranslation3D().y, -GetTranslation3D().z));

	Vec3 iBasis = Vec3(rotation.m_values[Ix], rotation.m_values[Jx], rotation.m_values[Kx]);
	Vec3 jBasis = Vec3(rotation.m_values[Iy], rotation.m_values[Jy], rotation.m_values[Ky]);
	Vec3 kBasis = Vec3(rotation.m_values[Iz], rotation.m_values[Jz], rotation.m_values[Kz]);
	Vec3 translation;

	translation.x = (iBasis.x * translationInverse.m_values[Tx]) + (jBasis.x * translationInverse.m_values[Ty]) + (kBasis.x * translationInverse.m_values[Tz]);
	translation.y = (iBasis.y * translationInverse.m_values[Tx]) + (jBasis.y * translationInverse.m_values[Ty]) + (kBasis.y * translationInverse.m_values[Tz]);
	translation.z = (iBasis.z * translationInverse.m_values[Tx]) + (jBasis.z * translationInverse.m_values[Ty]) + (kBasis.z * translationInverse.m_values[Tz]);

	Mat44 result = Mat44(iBasis, jBasis, kBasis, translation);

	return result;
}

void Mat44::SetTranslation2D(Vec2 const& translationXY)
{
	m_values[Tx] = translationXY.x;
	m_values[Ty] = translationXY.y;
	m_values[Tz] = 0;
	m_values[Tw] = 1;
}

void Mat44::SetTranslation3D(Vec3 const& translationXYZ)
{
	m_values[Tx] = translationXYZ.x;
	m_values[Ty] = translationXYZ.y;
	m_values[Tz] = translationXYZ.z;
	m_values[Tw] = 1;
}

void Mat44::SetIJ2D(Vec2 const& iBasis2D, Vec2 const& jBasis2D)
{
	m_values[Ix] = iBasis2D.x;
	m_values[Iy] = iBasis2D.y;
	m_values[Iz] = 0;
	m_values[Iw] = 0;
	m_values[Jx] = jBasis2D.x;
	m_values[Jy] = jBasis2D.y;
	m_values[Jz] = 0;
	m_values[Jw] = 0;
}

void Mat44::SetIJT2D(Vec2 const& iBasis2D, Vec2 const& jBasis2D, Vec2 const& translation2D)
{
	m_values[Ix] = iBasis2D.x;
	m_values[Iy] = iBasis2D.y;
	m_values[Iz] = 0;
	m_values[Iw] = 0;
	m_values[Jx] = jBasis2D.x;
	m_values[Jy] = jBasis2D.y;
	m_values[Jz] = 0;
	m_values[Jw] = 0;
	m_values[Tx] = translation2D.x;
	m_values[Ty] = translation2D.y;
	m_values[Tz] = 0;
	m_values[Tw] = 1;
}

void Mat44::SetIJK3D(Vec3 const& iBasis3D, Vec3 const& jBasis3D, Vec3 const& kBasis3D)
{
	m_values[Ix] = iBasis3D.x;
	m_values[Iy] = iBasis3D.y;
	m_values[Iz] = iBasis3D.z;
	m_values[Iw] = 0;
	m_values[Jx] = jBasis3D.x;
	m_values[Jy] = jBasis3D.y;
	m_values[Jz] = jBasis3D.z;
	m_values[Jw] = 0;
	m_values[Kx] = kBasis3D.x;
	m_values[Ky] = kBasis3D.y;
	m_values[Kz] = kBasis3D.z;
	m_values[Kw] = 0;
}

void Mat44::SetIJKT3D(Vec3 const& iBasis3D, Vec3 const& jBasis3D, Vec3 const& kBasis3D, Vec3 const& translation3D)
{
	m_values[Ix] = iBasis3D.x;
	m_values[Iy] = iBasis3D.y;
	m_values[Iz] = iBasis3D.z;
	m_values[Iw] = 0;
	m_values[Jx] = jBasis3D.x;
	m_values[Jy] = jBasis3D.y;
	m_values[Jz] = jBasis3D.z;
	m_values[Jw] = 0;
	m_values[Kx] = kBasis3D.x;
	m_values[Ky] = kBasis3D.y;
	m_values[Kz] = kBasis3D.z;
	m_values[Kw] = 0;
	m_values[Tx] = translation3D.x;
	m_values[Ty] = translation3D.y;
	m_values[Tz] = translation3D.z;
	m_values[Tw] = 1;
}

void Mat44::SetIJKT4D(Vec4 const& iBasis4D, Vec4 const& jBasis4D, Vec4 const& kBasis4D, Vec4 const& translation4D)
{
	m_values[Ix] = iBasis4D.x;
	m_values[Iy] = iBasis4D.y;
	m_values[Iz] = iBasis4D.z;
	m_values[Iw] = iBasis4D.w;
	m_values[Jx] = jBasis4D.x;
	m_values[Jy] = jBasis4D.y;
	m_values[Jz] = jBasis4D.z;
	m_values[Jw] = jBasis4D.w;
	m_values[Kx] = kBasis4D.x;
	m_values[Ky] = kBasis4D.y;
	m_values[Kz] = kBasis4D.z;
	m_values[Kw] = kBasis4D.w;
	m_values[Tx] = translation4D.x;
	m_values[Ty] = translation4D.y;
	m_values[Tz] = translation4D.z;
	m_values[Tw] = translation4D.w;
}

void Mat44::Transpose()
{
	Mat44 temp;

	temp.m_values[Ix] = m_values[Ix];
	temp.m_values[Iy] = m_values[Jx];
	temp.m_values[Iz] = m_values[Kx];
	temp.m_values[Iw] = m_values[Tx];
	temp.m_values[Jx] = m_values[Iy];
	temp.m_values[Jy] = m_values[Jy];
	temp.m_values[Jz] = m_values[Ky];
	temp.m_values[Jw] = m_values[Ty];
	temp.m_values[Kx] = m_values[Iz];
	temp.m_values[Ky] = m_values[Jz];
	temp.m_values[Kz] = m_values[Kz];
	temp.m_values[Kw] = m_values[Tz];
	temp.m_values[Tx] = m_values[Iw];
	temp.m_values[Ty] = m_values[Jw];
	temp.m_values[Tz] = m_values[Kw];
	temp.m_values[Tw] = m_values[Tw];

	*this = temp;
}

void Mat44::Orthonormalize_IFwd_JLeft_KUp()
{
	Vec3 iFwd = GetIBasis3D();
	Vec3 jLeft = GetJBasis3D();
	Vec3 kUp = GetKBasis3D();

	Vec3 jNormalized = jLeft.GetNormalized();

	Vec3 iNormal = iFwd - (DotProduct3D(iFwd, jNormalized) * jNormalized);
	Vec3 kNormal = kUp - (DotProduct3D(kUp, jNormalized) * jNormalized);
	
	Vec3 kNormalized = kNormal.GetNormalized();
	
	Vec3 ikNormal = iNormal - (DotProduct3D(iNormal, kNormalized) * kNormalized);

	Vec3 iNormalized = ikNormal.GetNormalized();

	SetIJK3D(iNormalized, jNormalized, kNormalized);
}

void Mat44::Append(Mat44 const& appendThis)
{
	Vec4 aX = Vec4(m_values[Ix], m_values[Jx], m_values[Kx], m_values[Tx]);
	Vec4 aY = Vec4(m_values[Iy], m_values[Jy], m_values[Ky], m_values[Ty]);
	Vec4 aZ = Vec4(m_values[Iz], m_values[Jz], m_values[Kz], m_values[Tz]);
	Vec4 aW = Vec4(m_values[Iw], m_values[Jw], m_values[Kw], m_values[Tw]);

	Vec4 bI = Vec4(appendThis.m_values[Ix], appendThis.m_values[Iy], appendThis.m_values[Iz], appendThis.m_values[Iw]);
	Vec4 bJ = Vec4(appendThis.m_values[Jx], appendThis.m_values[Jy], appendThis.m_values[Jz], appendThis.m_values[Jw]);
	Vec4 bK = Vec4(appendThis.m_values[Kx], appendThis.m_values[Ky], appendThis.m_values[Kz], appendThis.m_values[Kw]);
	Vec4 bT = Vec4(appendThis.m_values[Tx], appendThis.m_values[Ty], appendThis.m_values[Tz], appendThis.m_values[Tw]);

	m_values[Ix] = DotProduct2D(aX, bI);
	m_values[Iy] = DotProduct2D(aY, bI);
	m_values[Iz] = DotProduct2D(aZ, bI);
	m_values[Iw] = DotProduct2D(aW, bI);
	m_values[Jx] = DotProduct2D(aX, bJ);
	m_values[Jy] = DotProduct2D(aY, bJ);
	m_values[Jz] = DotProduct2D(aZ, bJ);
	m_values[Jw] = DotProduct2D(aW, bJ);
	m_values[Kx] = DotProduct2D(aX, bK);
	m_values[Ky] = DotProduct2D(aY, bK);
	m_values[Kz] = DotProduct2D(aZ, bK);
	m_values[Kw] = DotProduct2D(aW, bK);
	m_values[Tx] = DotProduct2D(aX, bT);
	m_values[Ty] = DotProduct2D(aY, bT);
	m_values[Tz] = DotProduct2D(aZ, bT);
	m_values[Tw] = DotProduct2D(aW, bT);
}

void Mat44::AppendZRotation(float degreesRotationAboutZ)
{
	Mat44 rotationMatrix;

	float c = CosDegrees(degreesRotationAboutZ);
	float s = SinDegrees(degreesRotationAboutZ);

	rotationMatrix.m_values[Ix] = c;
	rotationMatrix.m_values[Iy] = s;
	rotationMatrix.m_values[Iz] = 0;
	rotationMatrix.m_values[Iw] = 0;
	rotationMatrix.m_values[Jx] = -1.0f * s;
	rotationMatrix.m_values[Jy] = c;
	rotationMatrix.m_values[Jz] = 0;
	rotationMatrix.m_values[Jw] = 0;
	rotationMatrix.m_values[Kx] = 0;
	rotationMatrix.m_values[Ky] = 0;
	rotationMatrix.m_values[Kz] = 1;
	rotationMatrix.m_values[Kw] = 0;
	rotationMatrix.m_values[Tx] = 0;
	rotationMatrix.m_values[Ty] = 0;
	rotationMatrix.m_values[Tz] = 0;
	rotationMatrix.m_values[Tw] = 1;

	Append(rotationMatrix);
}

void Mat44::AppendYRotation(float degreesRotationAboutY)
{
	Mat44 rotationMatrix;

	float c = CosDegrees(degreesRotationAboutY);
	float s = SinDegrees(degreesRotationAboutY);

	rotationMatrix.m_values[Ix] = c;
	rotationMatrix.m_values[Iy] = 0;
	rotationMatrix.m_values[Iz] = -1.0f * s;
	rotationMatrix.m_values[Iw] = 0;
	rotationMatrix.m_values[Jx] = 0;
	rotationMatrix.m_values[Jy] = 1;
	rotationMatrix.m_values[Jz] = 0;
	rotationMatrix.m_values[Jw] = 0;
	rotationMatrix.m_values[Kx] = s;
	rotationMatrix.m_values[Ky] = 0;
	rotationMatrix.m_values[Kz] = c;
	rotationMatrix.m_values[Kw] = 0;
	rotationMatrix.m_values[Tx] = 0;
	rotationMatrix.m_values[Ty] = 0;
	rotationMatrix.m_values[Tz] = 0;
	rotationMatrix.m_values[Tw] = 1;

	Append(rotationMatrix);
}

void Mat44::AppendXRotation(float degreesRotationAboutX)
{
	Mat44 rotationMatrix;

	float c = CosDegrees(degreesRotationAboutX);
	float s = SinDegrees(degreesRotationAboutX);

	rotationMatrix.m_values[Ix] = 1;
	rotationMatrix.m_values[Iy] = 0;
	rotationMatrix.m_values[Iz] = 0;
	rotationMatrix.m_values[Iw] = 0;
	rotationMatrix.m_values[Jx] = 0;
	rotationMatrix.m_values[Jy] = c;
	rotationMatrix.m_values[Jz] = s;
	rotationMatrix.m_values[Jw] = 0;
	rotationMatrix.m_values[Kx] = 0;
	rotationMatrix.m_values[Ky] = -1.0f * s;
	rotationMatrix.m_values[Kz] = c;
	rotationMatrix.m_values[Kw] = 0;
	rotationMatrix.m_values[Tx] = 0;
	rotationMatrix.m_values[Ty] = 0;
	rotationMatrix.m_values[Tz] = 0;
	rotationMatrix.m_values[Tw] = 1;

	Append(rotationMatrix);
}

void Mat44::AppendTranslation2D(Vec2 const& translationXY)
{
	Mat44 rotationMatrix;

	rotationMatrix.m_values[Ix] = 1;
	rotationMatrix.m_values[Iy] = 0;
	rotationMatrix.m_values[Iz] = 0;
	rotationMatrix.m_values[Iw] = 0;
	rotationMatrix.m_values[Jx] = 0;
	rotationMatrix.m_values[Jy] = 1;
	rotationMatrix.m_values[Jz] = 0;
	rotationMatrix.m_values[Jw] = 0;
	rotationMatrix.m_values[Kx] = 0;
	rotationMatrix.m_values[Ky] = 0;
	rotationMatrix.m_values[Kz] = 1;
	rotationMatrix.m_values[Kw] = 0;
	rotationMatrix.m_values[Tx] = translationXY.x;
	rotationMatrix.m_values[Ty] = translationXY.y;
	rotationMatrix.m_values[Tz] = 0;
	rotationMatrix.m_values[Tw] = 1;

	Append(rotationMatrix);
}

void Mat44::AppendTranslation3D(Vec3 const& translationXYZ)
{
	Mat44 rotationMatrix;

	rotationMatrix.m_values[Ix] = 1;
	rotationMatrix.m_values[Iy] = 0;
	rotationMatrix.m_values[Iz] = 0;
	rotationMatrix.m_values[Iw] = 0;
	rotationMatrix.m_values[Jx] = 0;
	rotationMatrix.m_values[Jy] = 1;
	rotationMatrix.m_values[Jz] = 0;
	rotationMatrix.m_values[Jw] = 0;
	rotationMatrix.m_values[Kx] = 0;
	rotationMatrix.m_values[Ky] = 0;
	rotationMatrix.m_values[Kz] = 1;
	rotationMatrix.m_values[Kw] = 0;
	rotationMatrix.m_values[Tx] = translationXYZ.x;
	rotationMatrix.m_values[Ty] = translationXYZ.y;
	rotationMatrix.m_values[Tz] = translationXYZ.z;
	rotationMatrix.m_values[Tw] = 1;

	Append(rotationMatrix);
}

void Mat44::AppendScaleUniform2D(float uniformScaleXY)
{
	Mat44 rotationMatrix;

	rotationMatrix.m_values[Ix] = uniformScaleXY;
	rotationMatrix.m_values[Iy] = 0;
	rotationMatrix.m_values[Iz] = 0;
	rotationMatrix.m_values[Iw] = 0;
	rotationMatrix.m_values[Jx] = 0;
	rotationMatrix.m_values[Jy] = uniformScaleXY;
	rotationMatrix.m_values[Jz] = 0;
	rotationMatrix.m_values[Jw] = 0;
	rotationMatrix.m_values[Kx] = 0;
	rotationMatrix.m_values[Ky] = 0;
	rotationMatrix.m_values[Kz] = 1;
	rotationMatrix.m_values[Kw] = 0;
	rotationMatrix.m_values[Tx] = 0;
	rotationMatrix.m_values[Ty] = 0;
	rotationMatrix.m_values[Tz] = 0;
	rotationMatrix.m_values[Tw] = 1;

	Append(rotationMatrix);
}

void Mat44::AppendScaleUniform3D(float uniformScaleXYZ)
{
	Mat44 rotationMatrix;

	rotationMatrix.m_values[Ix] = uniformScaleXYZ;
	rotationMatrix.m_values[Iy] = 0;
	rotationMatrix.m_values[Iz] = 0;
	rotationMatrix.m_values[Iw] = 0;
	rotationMatrix.m_values[Jx] = 0;
	rotationMatrix.m_values[Jy] = uniformScaleXYZ;
	rotationMatrix.m_values[Jz] = 0;
	rotationMatrix.m_values[Jw] = 0;
	rotationMatrix.m_values[Kx] = 0;
	rotationMatrix.m_values[Ky] = 0;
	rotationMatrix.m_values[Kz] = uniformScaleXYZ;
	rotationMatrix.m_values[Kw] = 0;
	rotationMatrix.m_values[Tx] = 0;
	rotationMatrix.m_values[Ty] = 0;
	rotationMatrix.m_values[Tz] = 0;
	rotationMatrix.m_values[Tw] = 1;

	Append(rotationMatrix);
}

void Mat44::AppendScaleNonUniform2D(Vec2 const& nonUniformScaleXY)
{
	Mat44 rotationMatrix;

	rotationMatrix.m_values[Ix] = nonUniformScaleXY.x;
	rotationMatrix.m_values[Iy] = 0;
	rotationMatrix.m_values[Iz] = 0;
	rotationMatrix.m_values[Iw] = 0;
	rotationMatrix.m_values[Jx] = 0;
	rotationMatrix.m_values[Jy] = nonUniformScaleXY.y;
	rotationMatrix.m_values[Jz] = 0;
	rotationMatrix.m_values[Jw] = 0;
	rotationMatrix.m_values[Kx] = 0;
	rotationMatrix.m_values[Ky] = 0;
	rotationMatrix.m_values[Kz] = 1;
	rotationMatrix.m_values[Kw] = 0;
	rotationMatrix.m_values[Tx] = 0;
	rotationMatrix.m_values[Ty] = 0;
	rotationMatrix.m_values[Tz] = 0;
	rotationMatrix.m_values[Tw] = 1;

	Append(rotationMatrix);
}

void Mat44::AppendScaleNonUniform3D(Vec3 const& nonUniformScaleXYZ)
{
	Mat44 rotationMatrix;

	rotationMatrix.m_values[Ix] = nonUniformScaleXYZ.x;
	rotationMatrix.m_values[Iy] = 0;
	rotationMatrix.m_values[Iz] = 0;
	rotationMatrix.m_values[Iw] = 0;
	rotationMatrix.m_values[Jx] = 0;
	rotationMatrix.m_values[Jy] = nonUniformScaleXYZ.y;
	rotationMatrix.m_values[Jz] = 0;
	rotationMatrix.m_values[Jw] = 0;
	rotationMatrix.m_values[Kx] = 0;
	rotationMatrix.m_values[Ky] = 0;
	rotationMatrix.m_values[Kz] = nonUniformScaleXYZ.z;
	rotationMatrix.m_values[Kw] = 0;
	rotationMatrix.m_values[Tx] = 0;
	rotationMatrix.m_values[Ty] = 0;
	rotationMatrix.m_values[Tz] = 0;
	rotationMatrix.m_values[Tw] = 1;

	Append(rotationMatrix);
}
