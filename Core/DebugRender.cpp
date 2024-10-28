#include "Engine/Core/DebugRender.hpp"

#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/AABB2.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Core/Timer.hpp"
#include "Engine/Core/Clock.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Renderer/Camera.hpp"
#include "Engine/Renderer/Texture.hpp"
#include "Engine/Renderer/BitmapFont.hpp"
#include "Engine/Renderer/DX11Renderer.hpp"
#include "Engine/Renderer/VertexBuffer.hpp"
#include "Engine/Renderer/ConstantBuffer.hpp"

struct ModelConstants
{
	Vec4 ModelColor;
	Mat44 ModelMatrix;
};

struct DebugRenderGeometry
{
	std::vector<Vertex_PCU>		m_vertices;
	VertexBuffer*				m_gpuMesh					= nullptr;
	ConstantBuffer*				m_modelCBO					= nullptr;
	Timer*						m_timer						= nullptr;
	float						m_duration;
	Rgba8						m_startColor				= Rgba8::WHITE;
	Rgba8						m_endColor					= Rgba8::WHITE;
	Mat44						m_tranformMatrix			= Mat44();
	Texture*					m_texture					= nullptr;
	DebugRenderMode				m_mode;
	RasterizerMode				m_rasterizerMode;
	Vec3						m_billboardPos				= Vec3();
	bool						m_isBillboard				= false;

	DebugRenderGeometry();
	~DebugRenderGeometry();
};

struct ScreenRenderGeometry
{
	std::vector<Vertex_PCU>		m_vertices;
	VertexBuffer*				m_gpuMesh					= nullptr;
	Timer*						m_timer						= nullptr;
	ConstantBuffer*				m_modelCBO					= nullptr;
	float						m_duration;
	Rgba8						m_startColor				= Rgba8::WHITE;
	Rgba8						m_endColor					= Rgba8::WHITE;
	Mat44						m_tranformMatrix			= Mat44();
	Vec3						m_screenPos					= Vec3();
	float						m_screenHeight				= 0.0f;
	Texture*					m_texture					= nullptr;
	RasterizerMode				m_rasterizerMode;

	ScreenRenderGeometry();
	~ScreenRenderGeometry();
};

class DebugRender
{
public:
	std::vector<DebugRenderGeometry*>		m_debugPrimitives;
	std::vector<ScreenRenderGeometry*>		m_screenPrimitives;
	std::vector<ScreenRenderGeometry*>		m_infiniteMsgsPrimitives;
	std::vector<ScreenRenderGeometry*>		m_finiteMsgsPrimitives;
	DebugRenderConfig						m_config;
	BitmapFont*								m_font			= nullptr;
	bool									m_isVisible		= true;
	int										m_flag			= 0;
public:
											DebugRender()	= default;
											~DebugRender() {};
};

DebugRender* g_theDebugRender = nullptr;

void DebugRenderSystemStartup(DebugRenderConfig const& config)
{
	g_theDebugRender = new DebugRender();
	g_theDebugRender->m_config = config;

	g_theDebugRender->m_font = g_theDebugRender->m_config.m_renderer->CreateOrGetBitmapFont(g_theDebugRender->m_config.m_fontName.c_str());

	SubscribeEventCallbackFunction("DEBUGRENDERCLEAR", Command_DebugRenderClear);
	SubscribeEventCallbackFunction("DEBUGRENDERTOGGLE", Command_DebugRenderToggle);
}

void DebugRenderSystemShutdown()
{
	for (size_t index = 0; index < g_theDebugRender->m_debugPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_debugPrimitives[index])
		{
			if (g_theDebugRender->m_debugPrimitives[index]->m_duration == -1.0f)
			{
				delete g_theDebugRender->m_debugPrimitives[index];
				g_theDebugRender->m_debugPrimitives[index] = nullptr;
			}
		}
	}

	for (size_t index = 0; index < g_theDebugRender->m_screenPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_screenPrimitives[index])
		{
			if (g_theDebugRender->m_screenPrimitives[index]->m_duration == -1.0f)
			{
				delete g_theDebugRender->m_screenPrimitives[index];
				g_theDebugRender->m_screenPrimitives[index] = nullptr;
			}
		}
	}

	for (size_t index = 0; index < g_theDebugRender->m_finiteMsgsPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_finiteMsgsPrimitives[index])
		{
			if (g_theDebugRender->m_finiteMsgsPrimitives[index]->m_duration == -1.0f)
			{
				delete g_theDebugRender->m_finiteMsgsPrimitives[index];
				g_theDebugRender->m_finiteMsgsPrimitives[index] = nullptr;
			}
		}
	}

	for (size_t index = 0; index < g_theDebugRender->m_infiniteMsgsPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_infiniteMsgsPrimitives[index])
		{
			if (g_theDebugRender->m_infiniteMsgsPrimitives[index]->m_duration == -1.0f)
			{
				delete g_theDebugRender->m_infiniteMsgsPrimitives[index];
				g_theDebugRender->m_infiniteMsgsPrimitives[index] = nullptr;
			}
		}
	}

	g_theDebugRender->m_debugPrimitives.clear();
	g_theDebugRender->m_screenPrimitives.clear();
	g_theDebugRender->m_infiniteMsgsPrimitives.clear();
	g_theDebugRender->m_finiteMsgsPrimitives.clear();

	DELETE_PTR(g_theDebugRender);
}

void DebugRenderSetVisible()
{
	g_theDebugRender->m_isVisible = true;
}

void DebugRenderSetHidden()
{
	g_theDebugRender->m_isVisible = false;
}

void DebugRenderClear()
{
	g_theDebugRender->m_debugPrimitives.clear();
	g_theDebugRender->m_screenPrimitives.clear();
	g_theDebugRender->m_infiniteMsgsPrimitives.clear();
	g_theDebugRender->m_finiteMsgsPrimitives.clear();
}

void DebugRenderBeginFrame()
{

}

void DebugRenderWorld(Camera& camera)
{
	g_theDebugRender->m_config.m_renderer->BeginCamera(camera, RootSig::DEFAULT_PIPELINE);

	if (g_theDebugRender->m_isVisible)
	{
		for (size_t index = 0; index < g_theDebugRender->m_debugPrimitives.size(); index++)
		{
			if (g_theDebugRender->m_debugPrimitives[index])
			{
				Rgba8 debugPrimitiveColor = g_theDebugRender->m_debugPrimitives[index]->m_startColor;

				if (g_theDebugRender->m_debugPrimitives[index]->m_isBillboard)
				{
					g_theDebugRender->m_debugPrimitives[index]->m_tranformMatrix = GetBillboardMatrix(BillboardType::FULL_CAMERA_OPPOSING, camera.GetModelMatrix(), g_theDebugRender->m_debugPrimitives[index]->m_billboardPos);
				}

				if (g_theDebugRender->m_debugPrimitives[index]->m_duration > 0.0f)
				{
					debugPrimitiveColor = Interpolate(g_theDebugRender->m_debugPrimitives[index]->m_startColor, g_theDebugRender->m_debugPrimitives[index]->m_endColor, g_theDebugRender->m_debugPrimitives[index]->m_timer->GetElapsedFraction());
				}

				if (g_theDebugRender->m_debugPrimitives[index]->m_mode == DebugRenderMode::ALWAYS)
				{
					g_theDebugRender->m_config.m_renderer->SetModelConstants(RootSig::DEFAULT_PIPELINE, g_theDebugRender->m_debugPrimitives[index]->m_tranformMatrix, debugPrimitiveColor, g_theDebugRender->m_debugPrimitives[index]->m_modelCBO);

					g_theDebugRender->m_config.m_renderer->SetDepthMode(DepthMode::DISABLED);
					g_theDebugRender->m_config.m_renderer->SetBlendMode(BlendMode::ALPHA);
					g_theDebugRender->m_config.m_renderer->SetSamplerMode(SamplerMode::POINT_CLAMP);
					g_theDebugRender->m_config.m_renderer->SetRasterizerMode(g_theDebugRender->m_debugPrimitives[index]->m_rasterizerMode);
					g_theDebugRender->m_config.m_renderer->BindShader();
					g_theDebugRender->m_config.m_renderer->BindTexture(0, g_theDebugRender->m_debugPrimitives[index]->m_texture);
					g_theDebugRender->m_config.m_renderer->DrawVertexBuffer(g_theDebugRender->m_debugPrimitives[index]->m_gpuMesh, (int)g_theDebugRender->m_debugPrimitives[index]->m_vertices.size(), sizeof(Vertex_PCU));
				}
				else if (g_theDebugRender->m_debugPrimitives[index]->m_mode == DebugRenderMode::USE_DEPTH)
				{
					g_theDebugRender->m_config.m_renderer->SetModelConstants(RootSig::DEFAULT_PIPELINE, g_theDebugRender->m_debugPrimitives[index]->m_tranformMatrix, debugPrimitiveColor, g_theDebugRender->m_debugPrimitives[index]->m_modelCBO);

					g_theDebugRender->m_config.m_renderer->SetDepthMode(DepthMode::ENABLED);
					g_theDebugRender->m_config.m_renderer->SetBlendMode(BlendMode::ALPHA);
					g_theDebugRender->m_config.m_renderer->SetSamplerMode(SamplerMode::POINT_CLAMP);
					g_theDebugRender->m_config.m_renderer->SetRasterizerMode(g_theDebugRender->m_debugPrimitives[index]->m_rasterizerMode);
					g_theDebugRender->m_config.m_renderer->BindShader();
					g_theDebugRender->m_config.m_renderer->BindTexture(0, g_theDebugRender->m_debugPrimitives[index]->m_texture);
					g_theDebugRender->m_config.m_renderer->DrawVertexBuffer(g_theDebugRender->m_debugPrimitives[index]->m_gpuMesh, (int)g_theDebugRender->m_debugPrimitives[index]->m_vertices.size(), sizeof(Vertex_PCU));
				}
			}
		}

		for (size_t index = 0; index < g_theDebugRender->m_debugPrimitives.size(); index++)
		{
			if (g_theDebugRender->m_debugPrimitives[index])
			{
				Rgba8 debugPrimitiveColor = g_theDebugRender->m_debugPrimitives[index]->m_startColor;

				if (g_theDebugRender->m_debugPrimitives[index]->m_duration > 0.0f)
				{
					debugPrimitiveColor = Interpolate(g_theDebugRender->m_debugPrimitives[index]->m_startColor, g_theDebugRender->m_debugPrimitives[index]->m_endColor, g_theDebugRender->m_debugPrimitives[index]->m_timer->GetElapsedFraction());
				}

				if (g_theDebugRender->m_debugPrimitives[index]->m_mode == DebugRenderMode::X_RAY)
				{
					debugPrimitiveColor = Rgba8(255, 255, 224, 127);

					g_theDebugRender->m_config.m_renderer->SetModelConstants(RootSig::DEFAULT_PIPELINE, g_theDebugRender->m_debugPrimitives[index]->m_tranformMatrix, debugPrimitiveColor, g_theDebugRender->m_debugPrimitives[index]->m_modelCBO);

					g_theDebugRender->m_config.m_renderer->SetDepthMode(DepthMode::DISABLED);
					g_theDebugRender->m_config.m_renderer->SetBlendMode(BlendMode::ALPHA);
					g_theDebugRender->m_config.m_renderer->SetSamplerMode(SamplerMode::POINT_CLAMP);
					g_theDebugRender->m_config.m_renderer->SetRasterizerMode(g_theDebugRender->m_debugPrimitives[index]->m_rasterizerMode);
					g_theDebugRender->m_config.m_renderer->BindShader();
					g_theDebugRender->m_config.m_renderer->BindTexture(0, g_theDebugRender->m_debugPrimitives[index]->m_texture);
					g_theDebugRender->m_config.m_renderer->DrawVertexBuffer(g_theDebugRender->m_debugPrimitives[index]->m_gpuMesh, (int)g_theDebugRender->m_debugPrimitives[index]->m_vertices.size(), sizeof(Vertex_PCU));

					debugPrimitiveColor = g_theDebugRender->m_debugPrimitives[index]->m_startColor;

					g_theDebugRender->m_config.m_renderer->SetModelConstants(RootSig::DEFAULT_PIPELINE, g_theDebugRender->m_debugPrimitives[index]->m_tranformMatrix, debugPrimitiveColor, g_theDebugRender->m_debugPrimitives[index]->m_modelCBO);

					g_theDebugRender->m_config.m_renderer->SetDepthMode(DepthMode::ENABLED);
					g_theDebugRender->m_config.m_renderer->SetBlendMode(BlendMode::OPAQUE);
					g_theDebugRender->m_config.m_renderer->SetSamplerMode(SamplerMode::POINT_CLAMP);
					g_theDebugRender->m_config.m_renderer->SetRasterizerMode(g_theDebugRender->m_debugPrimitives[index]->m_rasterizerMode);
					g_theDebugRender->m_config.m_renderer->BindShader();
					g_theDebugRender->m_config.m_renderer->BindTexture(0, g_theDebugRender->m_debugPrimitives[index]->m_texture);
					g_theDebugRender->m_config.m_renderer->DrawVertexBuffer(g_theDebugRender->m_debugPrimitives[index]->m_gpuMesh, (int)g_theDebugRender->m_debugPrimitives[index]->m_vertices.size(), sizeof(Vertex_PCU));
				}
			}
		}
	}

	g_theDebugRender->m_config.m_renderer->EndCamera(camera);
}

void DebugRenderScreen(Camera& camera)
{
	g_theDebugRender->m_config.m_renderer->BeginCamera(camera, RootSig::DEFAULT_PIPELINE);

	for (size_t index = 0; index < g_theDebugRender->m_screenPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_screenPrimitives[index])
		{
			g_theDebugRender->m_config.m_renderer->SetModelConstants(RootSig::DEFAULT_PIPELINE, g_theDebugRender->m_screenPrimitives[index]->m_tranformMatrix, g_theDebugRender->m_screenPrimitives[index]->m_startColor, g_theDebugRender->m_screenPrimitives[index]->m_modelCBO);

			g_theDebugRender->m_config.m_renderer->BindShader();
			g_theDebugRender->m_config.m_renderer->SetDepthMode(DepthMode::ENABLED);
			g_theDebugRender->m_config.m_renderer->SetBlendMode(BlendMode::ALPHA);
			g_theDebugRender->m_config.m_renderer->SetSamplerMode(SamplerMode::POINT_CLAMP);
			g_theDebugRender->m_config.m_renderer->SetRasterizerMode(g_theDebugRender->m_screenPrimitives[index]->m_rasterizerMode);
			g_theDebugRender->m_config.m_renderer->BindTexture(0, g_theDebugRender->m_screenPrimitives[index]->m_texture);
			g_theDebugRender->m_config.m_renderer->DrawVertexBuffer(g_theDebugRender->m_screenPrimitives[index]->m_gpuMesh, (int)g_theDebugRender->m_screenPrimitives[index]->m_vertices.size(), sizeof(Vertex_PCU));
		}
	}

	for (size_t index = 0; index < g_theDebugRender->m_infiniteMsgsPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_infiniteMsgsPrimitives[index])
		{
			float yPos = 765.0f;

			yPos -= 15.0f * (g_theDebugRender->m_infiniteMsgsPrimitives.size() - (index + 1));

			Mat44 translation;
			translation.SetTranslation3D(Vec3(400.0f, yPos, 0.0f));

			g_theDebugRender->m_config.m_renderer->SetModelConstants(RootSig::DEFAULT_PIPELINE, g_theDebugRender->m_screenPrimitives[index]->m_tranformMatrix, g_theDebugRender->m_screenPrimitives[index]->m_startColor, g_theDebugRender->m_screenPrimitives[index]->m_modelCBO);

			g_theDebugRender->m_config.m_renderer->SetDepthMode(DepthMode::ENABLED);
			g_theDebugRender->m_config.m_renderer->SetBlendMode(BlendMode::ALPHA);
			g_theDebugRender->m_config.m_renderer->SetSamplerMode(SamplerMode::POINT_CLAMP);
			g_theDebugRender->m_config.m_renderer->SetRasterizerMode(g_theDebugRender->m_infiniteMsgsPrimitives[index]->m_rasterizerMode);
			g_theDebugRender->m_config.m_renderer->BindShader();
			g_theDebugRender->m_config.m_renderer->BindTexture(0, g_theDebugRender->m_infiniteMsgsPrimitives[index]->m_texture);
			g_theDebugRender->m_config.m_renderer->DrawVertexBuffer(g_theDebugRender->m_screenPrimitives[index]->m_gpuMesh, (int)g_theDebugRender->m_screenPrimitives[index]->m_vertices.size(), sizeof(Vertex_PCU));
		}
	}

	for (size_t index = 0; index < g_theDebugRender->m_finiteMsgsPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_finiteMsgsPrimitives[index])
		{
			float yPos = 765.0f;

			yPos -= 15.0f * (g_theDebugRender->m_finiteMsgsPrimitives.size() - (index + 1));

			Mat44 translation;
			translation.SetTranslation3D(Vec3(400.0f, yPos, 0.0f));

			g_theDebugRender->m_config.m_renderer->SetModelConstants(RootSig::DEFAULT_PIPELINE, g_theDebugRender->m_screenPrimitives[index]->m_tranformMatrix, g_theDebugRender->m_screenPrimitives[index]->m_startColor, g_theDebugRender->m_screenPrimitives[index]->m_modelCBO);

			g_theDebugRender->m_config.m_renderer->SetDepthMode(DepthMode::ENABLED);
			g_theDebugRender->m_config.m_renderer->SetBlendMode(BlendMode::ALPHA);
			g_theDebugRender->m_config.m_renderer->SetSamplerMode(SamplerMode::POINT_CLAMP);
			g_theDebugRender->m_config.m_renderer->SetRasterizerMode(g_theDebugRender->m_finiteMsgsPrimitives[index]->m_rasterizerMode);
			g_theDebugRender->m_config.m_renderer->BindShader();
			g_theDebugRender->m_config.m_renderer->BindTexture(0, g_theDebugRender->m_finiteMsgsPrimitives[index]->m_texture);
			g_theDebugRender->m_config.m_renderer->DrawVertexBuffer(g_theDebugRender->m_screenPrimitives[index]->m_gpuMesh, (int)g_theDebugRender->m_screenPrimitives[index]->m_vertices.size(), sizeof(Vertex_PCU));
		}
	}

	g_theDebugRender->m_config.m_renderer->EndCamera(camera);
}

void DebugRenderEndFrame()
{
	for (size_t index = 0; index < g_theDebugRender->m_debugPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_debugPrimitives[index])
		{
			if (g_theDebugRender->m_debugPrimitives[index]->m_duration != -1.0f)
			{
				if (g_theDebugRender->m_debugPrimitives[index]->m_timer->HasPeriodElapsed() || g_theDebugRender->m_debugPrimitives[index]->m_duration == 0.0f)
				{
					delete g_theDebugRender->m_debugPrimitives[index];
					g_theDebugRender->m_debugPrimitives[index] = nullptr;
				}
			}
		}
	}

	for (size_t index = 0; index < g_theDebugRender->m_screenPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_screenPrimitives[index])
		{
			if (g_theDebugRender->m_screenPrimitives[index]->m_duration != -1.0f)
			{
				if (g_theDebugRender->m_screenPrimitives[index]->m_timer->HasPeriodElapsed() || g_theDebugRender->m_screenPrimitives[index]->m_duration == 0.0f)
				{
					delete g_theDebugRender->m_screenPrimitives[index];
					g_theDebugRender->m_screenPrimitives[index] = nullptr;
				}
			}
		}
	}
	
	for (size_t index = 0; index < g_theDebugRender->m_finiteMsgsPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_finiteMsgsPrimitives[index])
		{
			if (g_theDebugRender->m_finiteMsgsPrimitives[index]->m_duration != -1.0f)
			{
				if (g_theDebugRender->m_finiteMsgsPrimitives[index]->m_timer->HasPeriodElapsed() || g_theDebugRender->m_finiteMsgsPrimitives[index]->m_duration == 0.0f)
				{
					delete g_theDebugRender->m_finiteMsgsPrimitives[index];
					g_theDebugRender->m_finiteMsgsPrimitives[index] = nullptr;
				}
			}
		}
	}
}

void DebugAddWorldPoint(Vec3 const& pos, float radius, float duration, Rgba8 const& startColor, Rgba8 const& endColor, DebugRenderMode mode)
{
	DebugRenderGeometry* primitive = new DebugRenderGeometry();

	AddVertsForSphere3D(primitive->m_vertices, pos, radius, Rgba8::WHITE, AABB2::ZERO_TO_ONE, 16);

	g_theDebugRender->m_config.m_renderer->CopyCPUToGPU(primitive->m_vertices.data(), (int)primitive->m_vertices.size() * sizeof(Vertex_PCU), primitive->m_gpuMesh);

	primitive->m_timer = new Timer(duration, &Clock::GetSystemClock());
	primitive->m_duration = duration;
	primitive->m_startColor = startColor;
	primitive->m_endColor = endColor;
	primitive->m_mode = mode;
	primitive->m_rasterizerMode = RasterizerMode::SOLID_CULL_BACK;

	for (size_t index = 0; index < g_theDebugRender->m_debugPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_debugPrimitives[index] == nullptr)
		{
			g_theDebugRender->m_debugPrimitives[index] = primitive;
			g_theDebugRender->m_debugPrimitives[index]->m_timer->Start();
			return;
		}
	}

	g_theDebugRender->m_debugPrimitives.push_back(primitive);
	g_theDebugRender->m_debugPrimitives.back()->m_timer->Start();
}

void DebugAddWorldLine(Vec3 const& start, Vec3 const& end, float radius, float duration, Rgba8 const& startColor, Rgba8 const& endColor, DebugRenderMode mode)
{
	DebugRenderGeometry* primitive = new DebugRenderGeometry();

	AddVertsForCylinder3D(primitive->m_vertices, start, end, radius, Rgba8::WHITE, AABB2::ZERO_TO_ONE, 16);

	g_theDebugRender->m_config.m_renderer->CopyCPUToGPU(primitive->m_vertices.data(), (int)primitive->m_vertices.size() * sizeof(Vertex_PCU), primitive->m_gpuMesh);

	primitive->m_timer = new Timer(duration, &Clock::GetSystemClock());
	primitive->m_duration = duration;
	primitive->m_startColor = startColor;
	primitive->m_endColor = endColor;
	primitive->m_mode = mode;
	primitive->m_rasterizerMode = RasterizerMode::SOLID_CULL_NONE;

	for (size_t index = 0; index < g_theDebugRender->m_debugPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_debugPrimitives[index] == nullptr)
		{
			g_theDebugRender->m_debugPrimitives[index] = primitive;
			g_theDebugRender->m_debugPrimitives[index]->m_timer->Start();
			return;
		}
	}

	g_theDebugRender->m_debugPrimitives.push_back(primitive);
	g_theDebugRender->m_debugPrimitives.back()->m_timer->Start();
}

void DebugAddWorldWireCylinder(Vec3 const& base, Vec3 const& top, float radius, float duration, Rgba8 const& startColor, Rgba8 const& endColor, DebugRenderMode mode)
{
	DebugRenderGeometry* primitive = new DebugRenderGeometry();

	AddVertsForCylinder3D(primitive->m_vertices, base, top, radius, Rgba8::WHITE, AABB2::ZERO_TO_ONE, 16);

	g_theDebugRender->m_config.m_renderer->CopyCPUToGPU(primitive->m_vertices.data(), (int)primitive->m_vertices.size() * sizeof(Vertex_PCU), primitive->m_gpuMesh);

	primitive->m_timer = new Timer(duration, &Clock::GetSystemClock());
	primitive->m_duration = duration;
	primitive->m_startColor = startColor;
	primitive->m_endColor = endColor;
	primitive->m_mode = mode;
	primitive->m_rasterizerMode = RasterizerMode::WIREFRAME_CULL_NONE;

	for (size_t index = 0; index < g_theDebugRender->m_debugPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_debugPrimitives[index] == nullptr)
		{
			g_theDebugRender->m_debugPrimitives[index] = primitive;
			g_theDebugRender->m_debugPrimitives[index]->m_timer->Start();
			return;
		}
	}

	g_theDebugRender->m_debugPrimitives.push_back(primitive);
	g_theDebugRender->m_debugPrimitives.back()->m_timer->Start();
}

void DebugAddWorldWireSphere(Vec3 const& center, float radius, float duration, Rgba8 const& startColor, Rgba8 const& endColor, DebugRenderMode mode)
{
	DebugRenderGeometry* primitive = new DebugRenderGeometry();

	AddVertsForSphere3D(primitive->m_vertices, center, radius, Rgba8::WHITE, AABB2::ZERO_TO_ONE);

	g_theDebugRender->m_config.m_renderer->CopyCPUToGPU(primitive->m_vertices.data(), (int)primitive->m_vertices.size() * sizeof(Vertex_PCU), primitive->m_gpuMesh);

	primitive->m_timer = new Timer(duration, &Clock::GetSystemClock());
	primitive->m_duration = duration;
	primitive->m_startColor = startColor;
	primitive->m_endColor = endColor;
	primitive->m_mode = mode;
	primitive->m_rasterizerMode = RasterizerMode::WIREFRAME_CULL_NONE;

	for (size_t index = 0; index < g_theDebugRender->m_debugPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_debugPrimitives[index] == nullptr)
		{
			g_theDebugRender->m_debugPrimitives[index] = primitive;
			g_theDebugRender->m_debugPrimitives[index]->m_timer->Start();
			return;
		}
	}

	g_theDebugRender->m_debugPrimitives.push_back(primitive);
	g_theDebugRender->m_debugPrimitives.back()->m_timer->Start();
}

void DebugAddWorldArrow(Vec3 const& start, Vec3 const& end, float radius, float duration, Rgba8 const& startColor, Rgba8 const& endColor, DebugRenderMode mode)
{
	DebugRenderGeometry* primitive = new DebugRenderGeometry();

	Vec3 cylinderEnd = start + (0.7f * (end - start));

	AddVertsForCylinder3D(primitive->m_vertices, start, cylinderEnd, radius, Rgba8::WHITE, AABB2::ZERO_TO_ONE, 16);
	AddVertsForCone3D(primitive->m_vertices, cylinderEnd, end, radius + (radius * 0.5f), Rgba8::WHITE, AABB2::ZERO_TO_ONE, 16);

	g_theDebugRender->m_config.m_renderer->CopyCPUToGPU(primitive->m_vertices.data(), (int)primitive->m_vertices.size() * sizeof(Vertex_PCU), primitive->m_gpuMesh);

	primitive->m_timer = new Timer(duration, &Clock::GetSystemClock());
	primitive->m_duration = duration;
	primitive->m_startColor = startColor;
	primitive->m_endColor = endColor;
	primitive->m_mode = mode;
	primitive->m_rasterizerMode = RasterizerMode::SOLID_CULL_BACK;

	for (size_t index = 0; index < g_theDebugRender->m_debugPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_debugPrimitives[index] == nullptr)
		{
			g_theDebugRender->m_debugPrimitives[index] = primitive;
			g_theDebugRender->m_debugPrimitives[index]->m_timer->Start();
			return;
		}
	}

	g_theDebugRender->m_debugPrimitives.push_back(primitive);
	g_theDebugRender->m_debugPrimitives.back()->m_timer->Start();
}

void DebugAddWorldWireArrow(Vec3 const& start, Vec3 const& end, float radius, float duration, Rgba8 const& startColor, Rgba8 const& endColor, DebugRenderMode mode)
{
	DebugRenderGeometry* primitive = new DebugRenderGeometry();

	Vec3 cylinderEnd = start + (0.7f * (end - start));

	AddVertsForCylinder3D(primitive->m_vertices, start, cylinderEnd, radius, Rgba8::WHITE, AABB2::ZERO_TO_ONE, 16);
	AddVertsForCone3D(primitive->m_vertices, cylinderEnd, end, radius + (radius * 0.5f), Rgba8::WHITE, AABB2::ZERO_TO_ONE, 16);

	g_theDebugRender->m_config.m_renderer->CopyCPUToGPU(primitive->m_vertices.data(), (int)primitive->m_vertices.size() * sizeof(Vertex_PCU), primitive->m_gpuMesh);

	primitive->m_timer = new Timer(duration, &Clock::GetSystemClock());
	primitive->m_duration = duration;
	primitive->m_startColor = startColor;
	primitive->m_endColor = endColor;
	primitive->m_mode = mode;
	primitive->m_rasterizerMode = RasterizerMode::WIREFRAME_CULL_BACK;

	for (size_t index = 0; index < g_theDebugRender->m_debugPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_debugPrimitives[index] == nullptr)
		{
			g_theDebugRender->m_debugPrimitives[index] = primitive;
			g_theDebugRender->m_debugPrimitives[index]->m_timer->Start();
			return;
		}
	}

	g_theDebugRender->m_debugPrimitives.push_back(primitive);
	g_theDebugRender->m_debugPrimitives.back()->m_timer->Start();
}

void DebugAddWorldBasis(Mat44 const& transform, float duration, float length, float radius, DebugRenderMode mode)
{
	DebugRenderGeometry* primitive = new DebugRenderGeometry();

	AddVertsForCylinder3D(primitive->m_vertices, Vec3(0.0f, 0.0f, 0.0f), Vec3(0.7f * length, 0.0f, 0.0f), radius, Rgba8::RED, AABB2::ZERO_TO_ONE, 32);
	AddVertsForCone3D(primitive->m_vertices, Vec3(0.7f * length, 0.0f, 0.0f), Vec3(length, 0.0f, 0.0f), radius + (radius * 0.3f), Rgba8::RED, AABB2::ZERO_TO_ONE, 32);

	AddVertsForCylinder3D(primitive->m_vertices, Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 0.7f * length, 0.0f), radius, Rgba8::GREEN, AABB2::ZERO_TO_ONE, 32);
	AddVertsForCone3D(primitive->m_vertices, Vec3(0.0f, 0.7f * length, 0.0f), Vec3(0.0f, length, 0.0f), radius + (radius * 0.3f), Rgba8::GREEN, AABB2::ZERO_TO_ONE, 32);

	AddVertsForCylinder3D(primitive->m_vertices, Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.7f * length), radius, Rgba8::BLUE, AABB2::ZERO_TO_ONE, 32);
	AddVertsForCone3D(primitive->m_vertices, Vec3(0.0f, 0.0f, 0.7f * length), Vec3(0.0f, 0.0f, length), radius + (radius * 0.3f), Rgba8::BLUE, AABB2::ZERO_TO_ONE, 32);

	AddVertsForSphere3D(primitive->m_vertices, Vec3(0.0f, 0.0f, 0.0f), radius + (radius * 0.7f), Rgba8::WHITE, AABB2::ZERO_TO_ONE, 16);

	g_theDebugRender->m_config.m_renderer->CopyCPUToGPU(primitive->m_vertices.data(), (int)primitive->m_vertices.size() * sizeof(Vertex_PCU), primitive->m_gpuMesh);

	primitive->m_timer = new Timer(duration, &Clock::GetSystemClock());
	primitive->m_duration = duration;
	primitive->m_startColor = Rgba8::WHITE;
	primitive->m_endColor = Rgba8::WHITE;
	primitive->m_mode = mode;
	primitive->m_tranformMatrix = transform;
	primitive->m_rasterizerMode = RasterizerMode::SOLID_CULL_BACK;

	for (size_t index = 0; index < g_theDebugRender->m_debugPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_debugPrimitives[index] == nullptr)
		{
			g_theDebugRender->m_debugPrimitives[index] = primitive;
			g_theDebugRender->m_debugPrimitives[index]->m_timer->Start();
			return;
		}
	}

	g_theDebugRender->m_debugPrimitives.push_back(primitive);
	g_theDebugRender->m_debugPrimitives.back()->m_timer->Start();
}

void DebugAddWorldText(std::string const& text, Mat44 const& transform, float textHeight, Vec2 const& alignment, float duration, Rgba8 const& startColor, Rgba8 const& endColor, DebugRenderMode mode)
{
	UNUSED(alignment);

	DebugRenderGeometry* primitive = new DebugRenderGeometry();

	g_theDebugRender->m_font->AddVertsForText3DAtOriginXForward(primitive->m_vertices, textHeight, text);

	g_theDebugRender->m_config.m_renderer->CopyCPUToGPU(primitive->m_vertices.data(), (int)primitive->m_vertices.size() * sizeof(Vertex_PCU), primitive->m_gpuMesh);

	primitive->m_timer = new Timer(duration, &Clock::GetSystemClock());
	primitive->m_duration = duration;
	primitive->m_startColor = startColor;
	primitive->m_endColor = endColor;
	primitive->m_mode = mode;
	primitive->m_tranformMatrix = transform;
	primitive->m_rasterizerMode = RasterizerMode::SOLID_CULL_NONE;
	primitive->m_texture = &g_theDebugRender->m_font->GetTexture();

	for (size_t index = 0; index < g_theDebugRender->m_debugPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_debugPrimitives[index] == nullptr)
		{
			g_theDebugRender->m_debugPrimitives[index] = primitive;
			g_theDebugRender->m_debugPrimitives[index]->m_timer->Start();
			return;
		}
	}

	g_theDebugRender->m_debugPrimitives.push_back(primitive);
	g_theDebugRender->m_debugPrimitives.back()->m_timer->Start();
}

void DebugAddWorldBillboardText(std::string const& text, Mat44 const& transform, float textHeight, Vec2 const& alignment, float duration, Rgba8 const& startColor, Rgba8 const& endColor, DebugRenderMode mode)
{
	UNUSED(alignment);
	UNUSED(mode);

	Vec3 billboardPos = transform.GetTranslation3D() + (transform.GetIBasis3D() * 2.0f);

	Mat44 billboardtransform = GetBillboardMatrix(BillboardType::FULL_CAMERA_OPPOSING, transform, billboardPos);
	
	DebugRenderGeometry* primitive = new DebugRenderGeometry();

	g_theDebugRender->m_font->AddVertsForText3DAtOriginXForward(primitive->m_vertices, textHeight, text);

	g_theDebugRender->m_config.m_renderer->CopyCPUToGPU(primitive->m_vertices.data(), (int)primitive->m_vertices.size() * sizeof(Vertex_PCU), primitive->m_gpuMesh);

	primitive->m_timer = new Timer(duration, &Clock::GetSystemClock());
	primitive->m_billboardPos = billboardPos;
	primitive->m_duration = duration;
	primitive->m_startColor = startColor;
	primitive->m_endColor = endColor;
	primitive->m_mode = mode;
	primitive->m_tranformMatrix = billboardtransform;
	primitive->m_isBillboard = true;
	primitive->m_rasterizerMode = RasterizerMode::SOLID_CULL_NONE;
	primitive->m_texture = &g_theDebugRender->m_font->GetTexture();

	for (size_t index = 0; index < g_theDebugRender->m_debugPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_debugPrimitives[index] == nullptr)
		{
			g_theDebugRender->m_debugPrimitives[index] = primitive;
			g_theDebugRender->m_debugPrimitives[index]->m_timer->Start();
			return;
		}
	}

	g_theDebugRender->m_debugPrimitives.push_back(primitive);
	g_theDebugRender->m_debugPrimitives.back()->m_timer->Start();
}

void DebugAddScreenText(std::string const& text, Mat44 const& transform, float textHeight, Vec2 const& alignment, float duration, Rgba8 const& startColor, Rgba8 const& endColor, DebugRenderMode mode)
{
	UNUSED(alignment);
	UNUSED(mode);

	ScreenRenderGeometry* screenPrimitive = new ScreenRenderGeometry();

	float textWidth = text.size() * textHeight;

	AABB2 bounds = AABB2(textWidth * -0.5f, textHeight * -0.5f, textWidth * 0.5f, textHeight * 0.5f);

	g_theDebugRender->m_font->AddVertsForTextInBox2D(screenPrimitive->m_vertices, bounds, textHeight, text, Rgba8::WHITE, 1.0f, alignment);

	g_theDebugRender->m_config.m_renderer->CopyCPUToGPU(screenPrimitive->m_vertices.data(), (int)screenPrimitive->m_vertices.size() * sizeof(Vertex_PCU), screenPrimitive->m_gpuMesh);

	screenPrimitive->m_timer = new Timer(duration, &Clock::GetSystemClock());
	screenPrimitive->m_duration = duration;
	screenPrimitive->m_startColor = startColor;
	screenPrimitive->m_endColor = endColor;
	screenPrimitive->m_tranformMatrix = transform;
	screenPrimitive->m_screenHeight = textHeight;
	screenPrimitive->m_rasterizerMode = RasterizerMode::SOLID_CULL_NONE;
	screenPrimitive->m_texture = &g_theDebugRender->m_font->GetTexture();

	for (size_t index = 0; index < g_theDebugRender->m_screenPrimitives.size(); index++)
	{
		if (g_theDebugRender->m_screenPrimitives[index] == nullptr)
		{
			g_theDebugRender->m_screenPrimitives[index] = screenPrimitive;
			g_theDebugRender->m_screenPrimitives[index]->m_timer->Start();
			return;
		}
	}

	g_theDebugRender->m_screenPrimitives.push_back(screenPrimitive);
	g_theDebugRender->m_screenPrimitives.back()->m_timer->Start();
}

void DebugAddMessage(std::string const& text, Vec3 const& screenPosition, float textHeight, Vec2 const& alignment, float duration, Rgba8 const& startColor, Rgba8 const& endColor, DebugRenderMode mode)
{
	UNUSED(alignment);
	UNUSED(mode);

	ScreenRenderGeometry* screenPrimitive = new ScreenRenderGeometry();

	float textWidth = text.size() * textHeight;

	AABB2 bounds = AABB2(textWidth * -0.5f, textHeight * -0.5f, textWidth * 0.5f, textHeight * 0.5f);

	g_theDebugRender->m_font->AddVertsForTextInBox2D(screenPrimitive->m_vertices, bounds, textHeight, text);

	g_theDebugRender->m_config.m_renderer->CopyCPUToGPU(screenPrimitive->m_vertices.data(), (int)screenPrimitive->m_vertices.size() * sizeof(Vertex_PCU), screenPrimitive->m_gpuMesh);

	screenPrimitive->m_timer = new Timer(duration, &Clock::GetSystemClock());
	screenPrimitive->m_duration = duration;
	screenPrimitive->m_startColor = startColor;
	screenPrimitive->m_endColor = endColor;
	screenPrimitive->m_screenPos = screenPosition;
	screenPrimitive->m_screenHeight = textHeight;
	screenPrimitive->m_rasterizerMode = RasterizerMode::SOLID_CULL_NONE;
	screenPrimitive->m_texture = &g_theDebugRender->m_font->GetTexture();

	if (duration == -1.0f)
	{
		for (size_t index = 0; index < g_theDebugRender->m_screenPrimitives.size(); index++)
		{
			if (g_theDebugRender->m_screenPrimitives[index] == nullptr)
			{
				g_theDebugRender->m_screenPrimitives[index] = screenPrimitive;
				g_theDebugRender->m_screenPrimitives[index]->m_timer->Start();
				return;
			}
		}

		g_theDebugRender->m_infiniteMsgsPrimitives.push_back(screenPrimitive);
		g_theDebugRender->m_infiniteMsgsPrimitives.back()->m_timer->Start();
	}
	else
	{
		for (size_t index = 0; index < g_theDebugRender->m_screenPrimitives.size(); index++)
		{
			if (g_theDebugRender->m_screenPrimitives[index] == nullptr)
			{
				g_theDebugRender->m_screenPrimitives[index] = screenPrimitive;
				g_theDebugRender->m_screenPrimitives[index]->m_timer->Start();
				return;
			}
		}

		g_theDebugRender->m_finiteMsgsPrimitives.push_back(screenPrimitive);
		g_theDebugRender->m_finiteMsgsPrimitives.back()->m_timer->Start();
	}
}

bool Command_DebugRenderClear(EventArgs& args)
{
	UNUSED(args);

	if (g_theConsole->IsOpen())
	{
		if (g_theDebugRender->m_debugPrimitives.size() == 0)
			return true;

		g_theDebugRender->m_debugPrimitives.clear();
	}

	return false;
}

bool Command_DebugRenderToggle(EventArgs& args)
{
	UNUSED(args);

	if (g_theConsole->IsOpen())
	{
		if (g_theDebugRender->m_isVisible)
		{
			DebugRenderSetHidden();
			return true;
		}
		else
		{
			DebugRenderSetVisible();
			return true;
		}
	}

	return false;
}

DebugRenderGeometry::DebugRenderGeometry()
{
	m_gpuMesh = g_theDebugRender->m_config.m_renderer->CreateVertexBuffer(sizeof(Vertex_PCU));
	m_modelCBO = g_theDebugRender->m_config.m_renderer->CreateConstantBuffer(sizeof(ModelConstants), std::wstring(L"Debug World"));
}

DebugRenderGeometry::~DebugRenderGeometry()
{
	DELETE_PTR(m_gpuMesh); 
	DELETE_PTR(m_modelCBO); 
	DELETE_PTR(m_timer);
}

ScreenRenderGeometry::ScreenRenderGeometry()
{
	m_gpuMesh = g_theDebugRender->m_config.m_renderer->CreateVertexBuffer(sizeof(Vertex_PCU));
	m_modelCBO = g_theDebugRender->m_config.m_renderer->CreateConstantBuffer(sizeof(ModelConstants), std::wstring(L"Debug Screen"));
}

ScreenRenderGeometry::~ScreenRenderGeometry()
{
	DELETE_PTR(m_gpuMesh); 
	DELETE_PTR(m_modelCBO); 
	DELETE_PTR(m_timer);
}
