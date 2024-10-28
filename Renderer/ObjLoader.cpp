#include "ObjLoader.hpp"

#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Core/FileUtils.hpp"
#include "Engine/Core/StringUtils.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Core/EngineCommon.hpp"

#include <filesystem>

bool ObjLoader::Load(std::string const& fileName, Mat44 const& transform, std::vector<Vertex_PCUTBN>& outVertices, std::vector<unsigned int>& outIndices, bool& outHasNormals, bool& outHasUVs)
{
	std::vector<Vec3> posArr;
	std::vector<Vec2> uvArr;
	std::vector<Vec3> normArr;
	std::vector<Face> faceArr;
	std::unordered_map<std::string, Rgba8> materialLibrary;

	std::string strr;
	std::string filePath = fileName;

	FileReadToString(strr, filePath);

	Strings st;

	std::string currentFaceMaterial;

	int w = SplitStringOnDelimiter(st, strr, "\r\n");

	if (w == 1)
	{
		for (int i = 0; i < (int)st.size(); i++)
		{
			ParsingMaterialFile(st[i], materialLibrary);
		}
	}
	else
	{
		w = SplitStringOnDelimiter(st, strr, "\n");

		if (w == 1)
		{
			for (int i = 0; i < (int)st.size(); i++)
			{
				ParsingMaterialFile(st[i], materialLibrary);
			}
		}
	}

	if (w == 1)
	{
		Strings faceMaterialInfo;

		for (int i = 0; i < (int)st.size(); i++)
		{
			ParsingVertexPos(st[i], posArr);
			ParsingVertexUVs(st[i], uvArr, outHasUVs);
			ParsingVertexNormals(st[i], normArr, outHasNormals);

			if (st[i].substr(0, 6) == "usemtl")
			{
				faceMaterialInfo = SplitStringOnDelimiter(st[i], ' ');
			}

			if (faceMaterialInfo.size() == 2)
			{
				ParsingFaces(st[i], faceArr, materialLibrary[faceMaterialInfo[1]]);
			}
			else
			{
				ParsingFaces(st[i], faceArr);
			}
		}
	}
	else
	{
		w = SplitStringOnDelimiter(st, strr, "\n");

		if (w == 1)
		{
			Strings faceMaterialInfo;

			for (int i = 0; i < (int)st.size(); i++)
			{
				ParsingVertexPos(st[i], posArr);
				ParsingVertexUVs(st[i], uvArr, outHasUVs);
				ParsingVertexNormals(st[i], normArr, outHasNormals);

				if (st[i].substr(0, 6) == "usemtl")
				{
					faceMaterialInfo = SplitStringOnDelimiter(st[i], ' ');
				}

				if (faceMaterialInfo.size() == 2)
				{
					ParsingFaces(st[i], faceArr, materialLibrary[faceMaterialInfo[1]]);
				}
				else
				{
					ParsingFaces(st[i], faceArr);
				}
			}
		}
	}

	GenerateVerticesAndIndices(faceArr, posArr, uvArr, normArr, outVertices, outIndices);

	TransformVertexArray3D(outVertices, transform, outHasNormals);

	return true;
}

#if DX12_RENDERER

bool ObjLoader::Load(std::string const& fileName, Mat44 const& transform, std::vector<MeshVertex_PCU>& outVertices, std::vector<unsigned int>& outIndices, bool& outHasNormals, bool& outHasUVs)
{
	UNUSED(transform);

	std::vector<Vec3> posArr;
	std::vector<Vec2> uvArr;
	std::vector<Vec3> normArr;
	std::vector<Face> faceArr;
	std::unordered_map<std::string, Rgba8> materialLibrary;

	std::string strr;
	std::string filePath = fileName;

	FileReadToString(strr, filePath);

	Strings st;

	std::string currentFaceMaterial;

	int w = SplitStringOnDelimiter(st, strr, "\r\n");

	if (w == 1)
	{
		for (int i = 0; i < st.size(); i++)
		{
			ParsingMaterialFile(st[i], materialLibrary);
		}
	}
	else
	{
		w = SplitStringOnDelimiter(st, strr, "\n");

		if (w == 1)
		{
			for (int i = 0; i < st.size(); i++)
			{
				ParsingMaterialFile(st[i], materialLibrary);
			}
		}
	}

	if (w == 1)
	{
		Strings faceMaterialInfo;

		for (int i = 0; i < st.size(); i++)
		{
			ParsingVertexPos(st[i], posArr);
			ParsingVertexUVs(st[i], uvArr, outHasUVs);
			ParsingVertexNormals(st[i], normArr, outHasNormals);

			if (st[i].substr(0, 6) == "usemtl")
			{
				faceMaterialInfo = SplitStringOnDelimiter(st[i], ' ');
			}

			if (faceMaterialInfo.size() == 2)
			{
				ParsingFaces(st[i], faceArr, materialLibrary[faceMaterialInfo[1]]);
			}
			else
			{
				ParsingFaces(st[i], faceArr);
			}
		}
	}
	else
	{
		w = SplitStringOnDelimiter(st, strr, "\n");

		if (w == 1)
		{
			Strings faceMaterialInfo;

			for (int i = 0; i < st.size(); i++)
			{
				ParsingVertexPos(st[i], posArr);
				ParsingVertexUVs(st[i], uvArr, outHasUVs);
				ParsingVertexNormals(st[i], normArr, outHasNormals);

				if (st[i].substr(0, 6) == "usemtl")
				{
					faceMaterialInfo = SplitStringOnDelimiter(st[i], ' ');
				}

				if (faceMaterialInfo.size() == 2)
				{
					ParsingFaces(st[i], faceArr, materialLibrary[faceMaterialInfo[1]]);
				}
				else
				{
					ParsingFaces(st[i], faceArr);
				}
			}
		}
	}

	GenerateVerticesAndIndices(faceArr, posArr, uvArr, normArr, outVertices, outIndices);

	TransformVertexArray3D(outVertices, transform, outHasNormals);

	return true;
}
bool ObjLoader::LoadXML(std::string const& fileName, Mat44 const& transform, std::vector<MeshVertex_PCU>& outVertices, std::vector<unsigned int>& outIndices, bool& outHasNormals, bool& outHasUVs)
{
	UNUSED(transform);

	tinyxml2::XMLDocument document;
	if (document.LoadFile(fileName.c_str()) == tinyxml2::XML_SUCCESS)
	{
		// Get the root element 
		tinyxml2::XMLElement* rootElement = document.RootElement();
		GUARANTEE_OR_DIE(rootElement, "XML couldn't be loaded"); 

		// Get the parent directory of the xml file
		std::filesystem::path xmlDirectory = std::filesystem::path(fileName).parent_path(); 

		// Get and process the OBJ relative path
		std::string objRelativeFilePath = ParseXmlAttribute(*rootElement, "path", std::string());
		std::filesystem::path objFileName = std::filesystem::path(objRelativeFilePath).filename(); // Extract the filename from the parsed xml
		std::filesystem::path objAbsPath = xmlDirectory / objFileName; // Combine the xml directory and the obj filename

		// Get and process the Material relative path, if present
		std::string materialRelativePath = ParseXmlAttribute(*rootElement, "material", std::string());

		//if (!materialRelativePath.empty())
		//{
		//	//m_material = new Material();
		//	if (!m_material->Load(materialRelativePath))
		//	{
		//		GUARANTEE_OR_DIE(false, "Invalid material in XML");
		//	}
		//}
		//else
		//{
		//	m_material = nullptr;
		//}

		Vec3 xBasis = ParseXmlAttribute(*rootElement->FirstChildElement("Transform"), "x", Vec3(1.f, 0.f, 0.f));
		Vec3 yBasis = ParseXmlAttribute(*rootElement->FirstChildElement("Transform"), "y", Vec3(1.f, 0.f, 0.f));
		Vec3 zBasis = ParseXmlAttribute(*rootElement->FirstChildElement("Transform"), "z", Vec3(1.f, 0.f, 0.f));
		Vec3 translation = ParseXmlAttribute(*rootElement->FirstChildElement("Transform"), "t", Vec3::ZERO);
		std::string scaleString = ParseXmlAttribute(*rootElement->FirstChildElement("Transform"), "scale", std::string());

		Mat44 transformMatrix;
		transformMatrix.SetIJKT3D(xBasis, yBasis, zBasis, translation);

		Strings scaleValues = SplitStringOnDelimiter(scaleString, ',');
		if (scaleValues.size() == 1)
		{
			// Use uniform scaling if there's only one value
			float uniformScale = std::stof(scaleValues[0]);
			transformMatrix.AppendScaleUniform3D(uniformScale);
		}
		else if (scaleValues.size() == 3)
		{
			// Use non-uniform scaling if there are three values
			float scaleX = std::stof(scaleValues[0]);
			float scaleY = std::stof(scaleValues[1]);
			float scaleZ = std::stof(scaleValues[2]);
			Vec3 nonUniformScale(scaleX, scaleY, scaleZ);
			transformMatrix.AppendScaleNonUniform3D(nonUniformScale);
		}
		else
		{
			GUARANTEE_OR_DIE(false, "Invalid scale format in XML");
		}

		Load(objAbsPath.string(), transformMatrix, outVertices, outIndices, outHasNormals, outHasUVs);
		return true;
	}
	return false;
}
#endif

void ObjLoader::ParsingVertexPos(std::string line, std::vector<Vec3>& positions)
{
	if (line[0] == 'v' && line[1] == ' ')
	{
		Strings positionStrings;

		std::string sug = line;

		int d = SplitStringOnDelimiter(positionStrings, sug, "  ");

		if (d == 1)
		{
			int c = SplitStringOnDelimiter(positionStrings, sug, "   ");

			if (c == 1)
			{
				positionStrings = SplitStringOnDelimiter(sug, ' ');

				float x = (float)std::atof(positionStrings[3].c_str());
				float y = (float)std::atof(positionStrings[4].c_str());
				float z = (float)std::atof(positionStrings[5].c_str());

				positions.push_back(Vec3(x, y, z));
			}
			else
			{
				positionStrings = SplitStringOnDelimiter(sug, ' ');

				float x = (float)std::atof(positionStrings[2].c_str());
				float y = (float)std::atof(positionStrings[3].c_str());
				float z = (float)std::atof(positionStrings[4].c_str());

				positions.push_back(Vec3(x, y, z));
			}
		}
		else
		{
			positionStrings = SplitStringOnDelimiter(sug, ' ');

			float x = (float)std::atof(positionStrings[1].c_str());
			float y = (float)std::atof(positionStrings[2].c_str());
			float z = (float)std::atof(positionStrings[3].c_str());

			positions.push_back(Vec3(x, y, z));
		}
	}
}

void ObjLoader::ParsingVertexUVs(std::string line, std::vector<Vec2>& uvs, bool& outHasUVs)
{
	if (line[0] == 'v' && line[1] == 't')
	{
		outHasUVs = true;

		Strings uv;

		std::string sug = line;

		int d = SplitStringOnDelimiter(uv, sug, "  ");
		int c = SplitStringOnDelimiter(uv, sug, "   ");

		if (d == 1)
		{
			uv = SplitStringOnDelimiter(sug, ' ');

			float u = (float)std::atof(uv[1].c_str());
			float v = (float)std::atof(uv[2].c_str());

			uvs.push_back(Vec2(u, v));
		}
		else if(c == 1)
		{
			uv = SplitStringOnDelimiter(sug, ' ');

			float u = (float)std::atof(uv[1].c_str());
			float v = (float)std::atof(uv[2].c_str());

			uvs.push_back(Vec2(u, v));
		}
		else
		{
			uv = SplitStringOnDelimiter(sug, ' ');

			float u = (float)std::atof(uv[1].c_str());
			float v = (float)std::atof(uv[2].c_str());

			uvs.push_back(Vec2(u, v));
		}
	}
}

void ObjLoader::ParsingVertexNormals(std::string line, std::vector<Vec3>& normals, bool& outHasNormals)
{
	if (line[0] == 'v' && line[1] == 'n')
	{
		outHasNormals = true;

		Strings normal;

		std::string sug = line;

		int d = SplitStringOnDelimiter(normal, sug, "  ");
		int c = SplitStringOnDelimiter(normal, sug, "   ");

		if (d == 1)
		{
			normal = SplitStringOnDelimiter(sug, ' ');

			float normX = (float)std::atof(normal[1].c_str());
			float normY = (float)std::atof(normal[2].c_str());
			float normZ = (float)std::atof(normal[3].c_str());

			normals.push_back(Vec3(normX, normY, normZ));
		}
		else if(c == 1)
		{
			normal = SplitStringOnDelimiter(sug, ' ');

			float normX = (float)std::atof(normal[1].c_str());
			float normY = (float)std::atof(normal[2].c_str());
			float normZ = (float)std::atof(normal[3].c_str());

			normals.push_back(Vec3(normX, normY, normZ));
		}
		else
		{
			normal = SplitStringOnDelimiter(sug, ' ');

			float normX = (float)std::atof(normal[1].c_str());
			float normY = (float)std::atof(normal[2].c_str());
			float normZ = (float)std::atof(normal[3].c_str());

			normals.push_back(Vec3(normX, normY, normZ));
		}
	}
}

void ObjLoader::ParsingFaces(std::string line, std::vector<Face>& outFaces, Rgba8 faceColor)
{
	if (line[0] == 'f')
	{
		Strings faces;

		std::string sug = line;

		faces = SplitStringOnDelimiter(sug, ' ');

		Face face;

		for (int j = 1; j < (int)faces.size(); j++)
		{
			Strings faceInfo;

			int d = SplitStringOnDelimiter(faceInfo, faces[j], "//");

			if (d == 1)
			{
				Vertex vert;

				if (faceInfo.size() == 2)
				{
					vert.m_v = std::atoi(faceInfo[0].c_str());
					vert.m_vt = -1;
					vert.m_vn = std::atoi(faceInfo[1].c_str());

					face.m_vertices.push_back(vert);
					continue;
				}
				else if (faceInfo.size() == 1)
				{
					vert.m_v = std::atoi(faceInfo[0].c_str());
					vert.m_vt = -1;
					vert.m_vn = -1;

					face.m_vertices.push_back(vert);
					continue;
				}
			}

			faceInfo = SplitStringOnDelimiter(faces[j], '/');

			if (faceInfo.size() == 1)
			{
				Vertex vert;

				vert.m_v = std::atoi(faceInfo[0].c_str());
				vert.m_vt = -1;
				vert.m_vn = -1;

				face.m_vertices.push_back(vert);
				continue;
			}
			else if (faceInfo.size() == 2)
			{
				Vertex vert;

				vert.m_v = std::atoi(faceInfo[0].c_str());
				vert.m_vt = std::atoi(faceInfo[1].c_str());
				vert.m_vn = -1;

				face.m_vertices.push_back(vert);
				continue;
			}
			else if (faceInfo.size() == 3)
			{
				Vertex vert;

				vert.m_v = std::atoi(faceInfo[0].c_str());
				vert.m_vt = std::atoi(faceInfo[1].c_str());
				vert.m_vn = std::atoi(faceInfo[2].c_str());

				face.m_vertices.push_back(vert);
				continue;
			}
		}

		face.m_color = faceColor;

		outFaces.push_back(face);
	}
}

void ObjLoader::ParsingMaterialFile(std::string line, std::unordered_map<std::string, Rgba8>& outMaterialLib)
{
	std::string matrialFile = line.substr(0, 6);

	if (matrialFile == "mtllib")
	{
		Strings materialInfo;

		std::string sug = line;

		materialInfo = SplitStringOnDelimiter(sug, ' ');

		Strings materialStrings;

		std::string materialString;
		std::string materialFilePath = "Data/Models/" + materialInfo[1];

		FileReadToString(materialString, materialFilePath);

		int materialSplit = SplitStringOnDelimiter(materialStrings, materialString, "\n");

		if (materialSplit == 1)
		{
			std::string materialName;

			for (int j = 0; j < (int)materialStrings.size(); j++)
			{
				if (materialStrings[j].substr(0, 6) == "newmtl")
				{
					Strings newMaterial;

					newMaterial = SplitStringOnDelimiter(materialStrings[j], ' ');

					materialName = newMaterial[1];
				}

				if (materialStrings[j].substr(0, 2) == "Kd")
				{
					Strings materialDiffuseColor;

					materialDiffuseColor = SplitStringOnDelimiter(materialStrings[j], ' ');

					Rgba8 color;

					float r = (float)std::atof(materialDiffuseColor[1].c_str());
					float g = (float)std::atof(materialDiffuseColor[2].c_str());
					float b = (float)std::atof(materialDiffuseColor[3].c_str());

					color.r  = unsigned char(r * 255);
					color.g  = unsigned char(g * 255);
					color.b  = unsigned char(b * 255);

					outMaterialLib[materialName] = color;
				}
			}
		}
	}
}

void ObjLoader::GenerateVerticesAndIndices(std::vector<Face> inFaces, std::vector<Vec3> positions, std::vector<Vec2> uvs, std::vector<Vec3> normals, std::vector<Vertex_PCUTBN>& outVertices, std::vector<unsigned int>& outIndices)
{
	int index = 0;

	for (int i = 0; i < (int)inFaces.size(); i++)
	{
		for (int j = 0; j < (int)inFaces[i].m_vertices.size(); j++)
		{
			Vertex faceVertex = inFaces[i].m_vertices[j];
			Rgba8 faceColor = inFaces[i].m_color;

			Vertex_PCUTBN vertex;

			vertex.m_position = positions[faceVertex.m_v - 1];
			vertex.m_color = faceColor;

			if (uvs.size() == 0)
			{
				vertex.m_uvTexCoords = Vec2::ZERO;
			}
			else
			{
				if (faceVertex.m_vt == -1)
				{
					vertex.m_uvTexCoords = Vec2::ZERO;
				}
				else
				{
					vertex.m_uvTexCoords = uvs[faceVertex.m_vt - 1];
				}
			}

			if (normals.size() == 0)
			{
				vertex.m_normal = Vec3::ZERO;
			}
			else
			{
				if (faceVertex.m_vn == -1)
				{
					vertex.m_normal = Vec3::ZERO;
				}
				else
				{
					vertex.m_normal = normals[faceVertex.m_vn - 1];
				}
			}

			outVertices.push_back(vertex);
		}

		for (int j = 0; j < (int)inFaces[i].m_vertices.size() - 2; j++)
		{
			unsigned int i0 = index + 0;
			unsigned int i1 = index + j + 1;
			unsigned int i2 = index + j + 2;

			outIndices.push_back(i0);
			outIndices.push_back(i1);
			outIndices.push_back(i2);
		}

		index += (int)inFaces[i].m_vertices.size();
	}

	//int index = 0;

	//std::vector<Triangle> triangles;

	//for (int i = 0; i < inFaces.size(); i++)
	//{
	//	Rgba8 faceColor = inFaces[i].m_color;

	//	for (int j = 0; j < inFaces[i].m_vertices.size(); j++)
	//	{
	//		Vertex faceVertex = inFaces[i].m_vertices[j];

	//		Vertex_PCUTBN vertex;
	//		vertex.m_position = positions[faceVertex.m_v - 1];
	//		vertex.m_color = faceColor;

	//		if (uvs.size() == 0)
	//		{
	//			vertex.m_uvTexCoords = Vec2::ZERO;
	//		}
	//		else
	//		{
	//			if (faceVertex.m_vt == -1)
	//			{
	//				vertex.m_uvTexCoords = Vec2::ZERO;
	//			}
	//			else
	//			{
	//				vertex.m_uvTexCoords = uvs[faceVertex.m_vt - 1];
	//			}
	//		}

	//		if (normals.size() == 0)
	//		{
	//			vertex.m_normal = Vec3::ZERO;
	//		}
	//		else
	//		{
	//			if (faceVertex.m_vn == -1)
	//			{
	//				vertex.m_normal = Vec3::ZERO;
	//			}
	//			else
	//			{
	//				vertex.m_normal = normals[faceVertex.m_vn - 1];
	//			}
	//		}

	//		outVertices.push_back(vertex);
	//	}

	//	for (int j = 0; j < inFaces[i].m_vertices.size() - 2; j++)
	//	{
	//		unsigned int i0 = index + 0;
	//		unsigned int i1 = index + j + 1;
	//		unsigned int i2 = index + j + 2;

	//		outIndices.push_back(i0);
	//		outIndices.push_back(i1);
	//		outIndices.push_back(i2);

	//		Triangle triangle((int)i0, (int)i1, (int)i2, faceColor);
	//		triangles.push_back(triangle);
	//	}

	//	index += (int)inFaces[i].m_vertices.size();
	//}

	//// Go through each triangle and make our own mesh data 
	//outVertices.reserve((int) (triangles.size() * 3));
	//outIndices.reserve((int) (triangles.size() * 3));
	//std::unordered_map<Vertex, int, VertexHash, VertexEqual> vertexMap;

	//for (size_t i = 0; i < triangles.size(); i++)
	//{
	//	Triangle& triangle = triangles[i];

	//	// Get Positions
	//	Vec3 const& p0 = positions[triangle.m_vertexPositionIndex[0]];
	//	Vec3 const& p1 = positions[triangle.m_vertexPositionIndex[1]];
	//	Vec3 const& p2 = positions[triangle.m_vertexPositionIndex[2]];

	//	// Get texture coordinates 
	//	Vec2 const& t0 = (triangle.m_vertexTextureCoordinateIndex[0] == -1 || triangle.m_vertexTextureCoordinateIndex[0] >= uvs.size()) ? Vec2::ZERO : uvs[triangle.m_vertexTextureCoordinateIndex[0]];
	//	Vec2 const& t1 = (triangle.m_vertexTextureCoordinateIndex[1] == -1 || triangle.m_vertexTextureCoordinateIndex[1] >= uvs.size()) ? Vec2::ZERO : uvs[triangle.m_vertexTextureCoordinateIndex[1]];
	//	Vec2 const& t2 = (triangle.m_vertexTextureCoordinateIndex[2] == -1 || triangle.m_vertexTextureCoordinateIndex[2] >= uvs.size()) ? Vec2::ZERO : uvs[triangle.m_vertexTextureCoordinateIndex[2]];

	//	// Get normals 
	//	Vec3 const& n0 = (triangle.m_vertexNormalIndex[0] == -1 || triangle.m_vertexNormalIndex[0] >= normals.size()) ? Vec3::ZERO : normals[triangle.m_vertexNormalIndex[0]];
	//	Vec3 const& n1 = (triangle.m_vertexNormalIndex[1] == -1 || triangle.m_vertexNormalIndex[1] >= normals.size()) ? Vec3::ZERO : normals[triangle.m_vertexNormalIndex[1]];
	//	Vec3 const& n2 = (triangle.m_vertexNormalIndex[2] == -1 || triangle.m_vertexNormalIndex[2] >= normals.size()) ? Vec3::ZERO : normals[triangle.m_vertexNormalIndex[2]];

	//	// Get color from the face
	//	Rgba8 const& c = triangle.m_color;

	//	// Create the vertices with the gathered data
	//	Vertex_PCUTBN gv0(Vec3(p0.x, p0.y, p0.z), Rgba8(c.r, c.g, c.b, c.a), Vec2(t0.x, t0.y), Vec3(0.f, 0.f, 0.f), Vec3(0.f, 0.f, 0.f), Vec3(n0.x, n0.y, n0.z));
	//	Vertex_PCUTBN gv1(Vec3(p1.x, p1.y, p1.z), Rgba8(c.r, c.g, c.b, c.a), Vec2(t1.x, t1.y), Vec3(0.f, 0.f, 0.f), Vec3(0.f, 0.f, 0.f), Vec3(n1.x, n1.y, n1.z));
	//	Vertex_PCUTBN gv2(Vec3(p2.x, p2.y, p2.z), Rgba8(c.r, c.g, c.b, c.a), Vec2(t2.x, t2.y), Vec3(0.f, 0.f, 0.f), Vec3(0.f, 0.f, 0.f), Vec3(n2.x, n2.y, n2.z));

	//	// Create corresponding vertex struct for the hash map
	//	Vertex v0 { triangle.m_vertexPositionIndex[0], triangle.m_vertexTextureCoordinateIndex[0], triangle.m_vertexNormalIndex[0] };
	//	Vertex v1 { triangle.m_vertexPositionIndex[1], triangle.m_vertexTextureCoordinateIndex[1], triangle.m_vertexNormalIndex[1] };
	//	Vertex v2 { triangle.m_vertexPositionIndex[2], triangle.m_vertexTextureCoordinateIndex[2], triangle.m_vertexNormalIndex[2] };

	//	int i0 = static_cast<unsigned int>(outVertices.size());
	//	auto it0 = vertexMap.find(v0);
	//	if (it0 == vertexMap.end())
	//	{
	//		outVertices.push_back(gv0);
	//		vertexMap[v0] = i0;
	//	}
	//	else
	//	{
	//		i0 = it0->second;
	//	}

	//	int i1 = static_cast<unsigned int>(outVertices.size());
	//	auto it1 = vertexMap.find(v1);
	//	if (it1 == vertexMap.end())
	//	{
	//		outVertices.push_back(gv1);
	//		vertexMap[v1] = i1;
	//	}
	//	else
	//	{
	//		i1 = it1->second;
	//	}

	//	int i2 = static_cast<unsigned int>(outVertices.size());
	//	auto it2 = vertexMap.find(v2);
	//	if (it2 == vertexMap.end())
	//	{
	//		outVertices.push_back(gv2);
	//		vertexMap[v2] = i2;
	//	}
	//	else
	//	{
	//		i2 = it2->second;
	//	}

	//	outIndices.push_back(i0);
	//	outIndices.push_back(i1);
	//	outIndices.push_back(i2);
	//}

	CalculateTangentSpaceBasisVectors(outVertices, outIndices, true, true);
}

#if DX12_RENDERER

void ObjLoader::GenerateVerticesAndIndices(std::vector<Face> inFaces, std::vector<Vec3> positions, std::vector<Vec2> uvs, std::vector<Vec3> normals, std::vector<MeshVertex_PCU>& outVertices, std::vector<unsigned int>& outIndices)
{
	Rgba8 color = Rgba8::WHITE;

	for(int i = 0; i < positions.size(); i++)
	{
		MeshVertex_PCU vertex;

		float vertColor[4];
		color.GetAsFloats(vertColor);

		vertex.m_position = positions[i];
		vertex.m_color = Vec4(vertColor[0], vertColor[1], vertColor[2], vertColor[3]);

		outVertices.push_back(vertex);
	}

	for (int i = 0; i < inFaces.size(); i++)
	{
		/*for (int j = 0; j < inFaces[i].m_vertices.size(); j++)
		{
			Vertex faceVertex = inFaces[i].m_vertices[j];
			Rgba8 faceColor = inFaces[i].m_color;

			MeshVertex_PCU vertex;

			vertex.m_position = positions[faceVertex.m_v - 1];
			vertex.m_color = Vec4::ONE;

			if (uvs.size() == 0)
			{
				vertex.m_uv = Vec2::ZERO;
			}
			else
			{
				if (faceVertex.m_vt == -1)
				{
					vertex.m_uv = Vec2::ZERO;
				}
				else
				{
					vertex.m_uv = uvs[faceVertex.m_vt - 1];
				}
			}

			outVertices.push_back(vertex);
		}*/

		for (int j = 0; j < inFaces[i].m_vertices.size() - 2; j++)
		{
			unsigned int i0 = inFaces[i].m_vertices[0    ].m_v - 1;
			unsigned int i1 = inFaces[i].m_vertices[j + 1].m_v - 1;
			unsigned int i2 = inFaces[i].m_vertices[j + 2].m_v - 1;

			outIndices.push_back(i0);
			outIndices.push_back(i1);
			outIndices.push_back(i2);
		}
	}
}
#endif