// main.cpp
// Milestone 4: Gerstner waves, world-space water sampler with inversion,
// buoyant rigid-body crate (Euler equations), fixed 120 Hz timestep.

#include <windows.h>
#include <d3d12.h>
#include <dxgi1_6.h>
#include <d3dcompiler.h>
#include <wrl/client.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#pragma comment(lib, "user32.lib")
#pragma comment(lib, "d3d12.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")

using Microsoft::WRL::ComPtr;

namespace {

    constexpr UINT kBackBufferCount = 2;
    constexpr UINT kWidth = 1280;
    constexpr UINT kHeight = 720;

    constexpr int   kCells = 128;
    constexpr float kHalf = 10.0f;
    constexpr float kGravity = 9.81f;
    constexpr float kRhoWater = 1000.0f;
    constexpr float kSimDt = 1.0f / 120.0f;
    constexpr float kSteepness = 0.80f;

    struct Vertex { float px, py, pz, nx, ny, nz; };

    struct WaveRecipe { float dirX, dirZ, wavelength, amp, phase0; };
    constexpr WaveRecipe kWaveRecipes[] = {
        {  1.00f, 0.15f, 7.0f, 0.32f, 0.0f },
        {  0.80f, 0.60f, 4.3f, 0.18f, 1.7f },
        {  0.35f, 1.00f, 2.6f, 0.10f, 3.1f },
        { -0.25f, 0.95f, 1.6f, 0.05f, 4.2f },
    };
    constexpr int kNumWaves = 4;

    struct WaveRuntime { float dirX, dirZ, k, omega, amp, phase0, q; };
    WaveRuntime g_waves[kNumWaves];

    // ---------------------------------------------------------------- math types
    struct Vec3 { float x, y, z; };
    inline Vec3 operator+(Vec3 a, Vec3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
    inline Vec3 operator-(Vec3 a, Vec3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
    inline Vec3 operator*(Vec3 a, float s) { return { a.x * s, a.y * s, a.z * s }; }
    inline float Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline Vec3 Cross(Vec3 a, Vec3 b)
    {
        return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }
    inline float Length(Vec3 v) { return std::sqrt(Dot(v, v)); }
    inline Vec3 Normalize(Vec3 v) { const float l = Length(v); return v * (1.0f / l); }

    struct Quat { float x, y, z, w; };
    inline Quat QuatMul(Quat a, Quat b)
    {
        return { a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                 a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                 a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                 a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z };
    }
    inline Quat QuatFromAxisAngle(Vec3 axis, float angle)
    {
        const float s = std::sin(angle * 0.5f);
        return { axis.x * s, axis.y * s, axis.z * s, std::cos(angle * 0.5f) };
    }
    inline Quat QuatNormalize(Quat q)
    {
        const float l = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        return { q.x / l, q.y / l, q.z / l, q.w / l };
    }
    inline void QuatAxes(Quat q, Vec3& X, Vec3& Y, Vec3& Z)
    {
        X = { 1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.w * q.z), 2 * (q.x * q.z - q.w * q.y) };
        Y = { 2 * (q.x * q.y - q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z + q.w * q.x) };
        Z = { 2 * (q.x * q.z + q.w * q.y), 2 * (q.y * q.z - q.w * q.x), 1 - 2 * (q.x * q.x + q.y * q.y) };
    }

    void Mat4Multiply(const float a[16], const float b[16], float out[16])
    {
        float t[16];
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                t[i * 4 + j] = a[i * 4 + 0] * b[0 * 4 + j] + a[i * 4 + 1] * b[1 * 4 + j]
                + a[i * 4 + 2] * b[2 * 4 + j] + a[i * 4 + 3] * b[3 * 4 + j];
        for (int i = 0; i < 16; ++i) out[i] = t[i];
    }

    void Mat4PerspectiveLH(float fovY, float aspect, float zn, float zf, float out[16])
    {
        const float yScale = 1.0f / std::tan(fovY * 0.5f);
        const float xScale = yScale / aspect;
        for (int i = 0; i < 16; ++i) out[i] = 0.0f;
        out[0] = xScale;
        out[5] = yScale;
        out[10] = zf / (zf - zn);
        out[11] = 1.0f;
        out[14] = -zn * zf / (zf - zn);
    }

    void Mat4LookAtLH(Vec3 eye, Vec3 target, Vec3 up, float out[16])
    {
        const Vec3 z = Normalize(target - eye);
        const Vec3 x = Normalize(Cross(up, z));
        const Vec3 y = Cross(z, x);
        out[0] = x.x;  out[1] = y.x;  out[2] = z.x;  out[3] = 0.0f;
        out[4] = x.y;  out[5] = y.y;  out[6] = z.y;  out[7] = 0.0f;
        out[8] = x.z;  out[9] = y.z;  out[10] = z.z; out[11] = 0.0f;
        out[12] = -Dot(x, eye); out[13] = -Dot(y, eye); out[14] = -Dot(z, eye); out[15] = 1.0f;
    }

    void Mat4FromQuatPos(Quat q, Vec3 p, float out[16])
    {
        Vec3 X, Y, Z;
        QuatAxes(q, X, Y, Z);
        out[0] = X.x;  out[1] = X.y;  out[2] = X.z;  out[3] = 0.0f;
        out[4] = Y.x;  out[5] = Y.y;  out[6] = Y.z;  out[7] = 0.0f;
        out[8] = Z.x;  out[9] = Z.y;  out[10] = Z.z; out[11] = 0.0f;
        out[12] = p.x; out[13] = p.y; out[14] = p.z; out[15] = 1.0f;
    }

    double NowSeconds()
    {
        using namespace std::chrono;
        return duration<double>(steady_clock::now().time_since_epoch()).count();
    }

    // ---------------------------------------------------------------- water field
    void InitWaves()
    {
        for (int i = 0; i < kNumWaves; ++i)
        {
            const auto& r = kWaveRecipes[i];
            const float len = std::sqrt(r.dirX * r.dirX + r.dirZ * r.dirZ);
            WaveRuntime& w = g_waves[i];
            w.dirX = r.dirX / len;
            w.dirZ = r.dirZ / len;
            w.k = 6.28318530718f / r.wavelength;
            w.omega = std::sqrt(kGravity * w.k);              // deep-water dispersion
            w.amp = r.amp;
            w.phase0 = r.phase0;
            // Steepness budget: sum(q_i * k_i * A_i) == kSteepness < 1 avoids loops.
            w.q = kSteepness / (w.k * w.amp * (float)kNumWaves);
        }
        std::printf("[water] Gerstner waves, steepness %.2f, sampler inverts horizontal shift.\n", kSteepness);
    }

    // Gerstner evaluation at MATERIAL coordinate p: returns displaced pos + normal.
    void GerstnerAt(float px, float pz, float t, Vec3* outPos, Vec3* outNormal)
    {
        float sx = 0, sy = 0, sz = 0;
        float nx = 0, ny = 1, nz = 0;
        for (int i = 0; i < kNumWaves; ++i)
        {
            const WaveRuntime& w = g_waves[i];
            const float theta = w.k * (w.dirX * px + w.dirZ * pz) - w.omega * t + w.phase0;
            const float s = std::sin(theta);
            const float c = std::cos(theta);
            sx += w.q * w.amp * w.dirX * c;
            sz += w.q * w.amp * w.dirZ * c;
            sy += w.amp * s;
            nx -= w.k * w.amp * w.dirX * c;
            ny -= w.q * w.k * w.amp * s;
            nz -= w.k * w.amp * w.dirZ * c;
        }
        *outPos = { px + sx, sy, pz + sz };
        *outNormal = Normalize({ nx, ny, nz });
    }

    // World-space query: invert the horizontal displacement by fixed-point
    // iteration, then evaluate. This is what buoyancy calls every tick.
    void SampleWaterWorld(float wx, float wz, float t, float* outHeight, Vec3* outNormal)
    {
        float px = wx, pz = wz;
        for (int it = 0; it < 3; ++it)
        {
            float sx = 0, sz = 0;
            for (int i = 0; i < kNumWaves; ++i)
            {
                const WaveRuntime& w = g_waves[i];
                const float theta = w.k * (w.dirX * px + w.dirZ * pz) - w.omega * t + w.phase0;
                const float c = std::cos(theta);
                sx += w.q * w.amp * w.dirX * c;
                sz += w.q * w.amp * w.dirZ * c;
            }
            px = wx - sx;
            pz = wz - sz;
        }
        Vec3 pos, n;
        GerstnerAt(px, pz, t, &pos, &n);
        *outHeight = pos.y;
        *outNormal = n;
    }

    // ---------------------------------------------------------------- rigid body
    struct HullSample { Vec3 local; float weight; };
    struct Body
    {
        Vec3 pos, vel, angVel;
        Quat q;
        float mass;
        Vec3 inertia;          // body-space diagonal (box)
        float halfW, halfH, halfD;
        HullSample samples[5];
    };

    Body g_body;
    float g_simTime = 0.0f;
    float g_physAcc = 0.0f;
    uint32_t g_randState = 123456789u;

    float Rand01()
    {
        g_randState = g_randState * 1664525u + 1013904223u;
        return (float)(g_randState >> 8) / 16777216.0f;
    }

    void DropBody()
    {
        Body& b = g_body;
        b.pos = { 1.5f, 1.6f, 1.0f };
        b.vel = { 0, 0, 0 };
        b.angVel = { (Rand01() - 0.5f) * 1.5f, (Rand01() - 0.5f) * 1.5f, (Rand01() - 0.5f) * 1.5f };
        b.q = QuatNormalize(QuatMul(QuatFromAxisAngle({ 1, 0, 0 }, (Rand01() - 0.5f) * 0.6f),
            QuatFromAxisAngle({ 0, 0, 1 }, (Rand01() - 0.5f) * 0.6f)));
        std::printf("[body] dropped at t=%.2f s\n", g_simTime);
    }

    void InitBody()
    {
        const float w = 0.9f, h = 0.7f, d = 0.9f;
        const float volume = w * h * d;
        Body& b = g_body;
        b.halfW = w * 0.5f; b.halfH = h * 0.5f; b.halfD = d * 0.5f;
        b.mass = kRhoWater * volume * 0.45f;    // floats ~45% submerged
        b.inertia = { b.mass / 12.0f * (h * h + d * d),
                      b.mass / 12.0f * (w * w + d * d),
                      b.mass / 12.0f * (w * w + h * h) };
        const float y = -b.halfH;
        b.samples[0] = { { -b.halfW * 0.8f, y, -b.halfD * 0.8f }, 0.18f };
        b.samples[1] = { {  b.halfW * 0.8f, y, -b.halfD * 0.8f }, 0.18f };
        b.samples[2] = { {  b.halfW * 0.8f, y,  b.halfD * 0.8f }, 0.18f };
        b.samples[3] = { { -b.halfW * 0.8f, y,  b.halfD * 0.8f }, 0.18f };
        b.samples[4] = { { 0.0f, y, 0.0f }, 0.28f };
        std::printf("[body] crate %.1fx%.1fx%.1f m, mass %.0f kg, target submersion 45%%. R = redrop.\n",
            w, h, d, b.mass);
        DropBody();
    }



    void StepBody(float dt)
    {
        Body& b = g_body;
        Vec3 X, Y, Z;
        QuatAxes(b.q, X, Y, Z);

        Vec3 F = { 0.0f, -kGravity * b.mass, 0.0f };
        Vec3 T = { 0, 0, 0 };

        for (int i = 0; i < 5; ++i)
        {
            const Vec3& s = b.samples[i].local;
            const Vec3 r = X * s.x + Y * s.y + Z * s.z;
            const Vec3 wpos = b.pos + r;

            float wh = 0; Vec3 wn;
            SampleWaterWorld(wpos.x, wpos.z, g_simTime, &wh, &wn);
            const float depth = wh - wpos.y;
            if (depth <= 0.0f) continue;

            const float frac = depth < (b.halfH * 2.0f) ? depth / (b.halfH * 2.0f) : 1.0f;
            const float volume = (b.halfW * 2) * (b.halfH * 2) * (b.halfD * 2);
            Vec3 Fs = { 0.0f, kRhoWater * kGravity * volume * b.samples[i].weight * frac, 0.0f };

            // Drag at the sample point (includes rotational velocity -> angular damping).
            const Vec3 vpt = b.vel + Cross(b.angVel, r);
            const float speed = Length(vpt);
            const float c = (90.0f + 25.0f * speed) * b.samples[i].weight;
            Fs = Fs - vpt * c;

            F = F + Fs;
            T = T + Cross(r, Fs);
        }

        // Linear integrate (semi-implicit Euler).
        b.vel = b.vel + F * (dt / b.mass);
        if (Length(b.vel) > 30.0f) b.vel = Normalize(b.vel) * 30.0f;
        b.pos = b.pos + b.vel * dt;

        // Rotational integrate: Euler's equations in BODY space.
        const Vec3 tb = { Dot(T, X), Dot(T, Y), Dot(T, Z) };
        Vec3 wb = { Dot(b.angVel, X), Dot(b.angVel, Y), Dot(b.angVel, Z) };
        const Vec3 Iw = { wb.x * b.inertia.x, wb.y * b.inertia.y, wb.z * b.inertia.z };
        const Vec3 gyro = Cross(wb, Iw);
        Vec3 alpha = { (tb.x - gyro.x) / b.inertia.x,
                       (tb.y - gyro.y) / b.inertia.y,
                       (tb.z - gyro.z) / b.inertia.z };
        wb = wb + alpha * dt;
        wb = wb * std::exp(-0.5f * dt);
        if (Length(wb) > 12.0f) wb = Normalize(wb) * 12.0f;
        b.angVel = X * wb.x + Y * wb.y + Z * wb.z;

        const Quat wq = { b.angVel.x, b.angVel.y, b.angVel.z, 0.0f };
        const Quat dq = QuatMul(wq, b.q);
        b.q = QuatNormalize({ b.q.x + dq.x * 0.5f * dt, b.q.y + dq.y * 0.5f * dt,
                              b.q.z + dq.z * 0.5f * dt, b.q.w + dq.w * 0.5f * dt });
    }

    // ---------------------------------------------------------------- graphics
    struct Graphics
    {
        ComPtr<ID3D12Device>              device;
        ComPtr<ID3D12CommandQueue>        queue;
        ComPtr<ID3D12CommandAllocator>    allocator;
        ComPtr<ID3D12GraphicsCommandList> cmdList;
        ComPtr<IDXGISwapChain3>           swapChain;
        ComPtr<ID3D12DescriptorHeap>      rtvHeap;
        ComPtr<ID3D12DescriptorHeap>      dsvHeap;
        ComPtr<ID3D12Resource>            depthTex;
        ComPtr<ID3D12RootSignature>       rootSig;
        ComPtr<ID3D12PipelineState>       psoSolid;
        ComPtr<ID3D12PipelineState>       psoWire;
        ComPtr<ID3D12PipelineState>       psoCrate;
        ComPtr<ID3D12Resource>            vertexBuffer;
        ComPtr<ID3D12Resource>            indexBuffer;
        ComPtr<ID3D12Resource>            crateVB;
        ComPtr<ID3D12Resource>            crateIB;
        ComPtr<ID3D12Fence>               fence;
        D3D12_VERTEX_BUFFER_VIEW          vbv{};
        D3D12_INDEX_BUFFER_VIEW           ibv{};
        D3D12_VERTEX_BUFFER_VIEW          cvbv{};
        D3D12_INDEX_BUFFER_VIEW           cibv{};
        HANDLE fenceEvent = nullptr;
        void* vbMapped = nullptr;
        UINT64 fenceValue = 0;
        UINT   rtvIncrement = 0;
        UINT   frameIndex = 0;
        UINT   indexCount = 0;
        bool   wireframe = false;
        bool   prevF1 = false;
        bool   prevR = false;
        float  view[16] = {}, proj[16] = {};
        float  rootConstants[56] = {};
        std::vector<float> baseX, baseZ;
    } g;

    LRESULT CALLBACK WindowProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
    {
        switch (msg)
        {
        case WM_DESTROY: PostQuitMessage(0); return 0;
        default: return DefWindowProcW(hwnd, msg, wParam, lParam);
        }
    }

    ComPtr<ID3DBlob> CompileShader(const wchar_t* path, const char* entry, const char* target)
    {
        ComPtr<ID3DBlob> code, errors;
        const HRESULT hr = D3DCompileFromFile(path, nullptr, nullptr, entry, target,
            D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION,
            0, &code, &errors);
        if (FAILED(hr))
        {
            if (errors) std::printf("[shader] %s\n", (const char*)errors->GetBufferPointer());
            return nullptr;
        }
        return code;
    }

    ComPtr<ID3D12Resource> CreateUploadBuffer(ID3D12Device* dev, const void* data, UINT64 size)
    {
        D3D12_HEAP_PROPERTIES hp{};
        hp.Type = D3D12_HEAP_TYPE_UPLOAD;
        D3D12_RESOURCE_DESC rd{};
        rd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
        rd.Width = size;
        rd.Height = 1;
        rd.DepthOrArraySize = 1;
        rd.MipLevels = 1;
        rd.SampleDesc.Count = 1;
        rd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

        ComPtr<ID3D12Resource> res;
        if (FAILED(dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd,
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr, IID_PPV_ARGS(&res))))
            return nullptr;
        if (data)
        {
            void* mapped = nullptr;
            res->Map(0, nullptr, &mapped);
            memcpy(mapped, data, (size_t)size);
            res->Unmap(0, nullptr);
        }
        return res;
    }

    ComPtr<ID3D12PipelineState> MakePso(ID3DBlob* vs, ID3DBlob* ps, D3D12_FILL_MODE fill)
    {
        D3D12_INPUT_ELEMENT_DESC layout[] = {
            { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0,  D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
            { "NORMAL",   0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        };
        D3D12_GRAPHICS_PIPELINE_STATE_DESC pd{};
        pd.pRootSignature = g.rootSig.Get();
        pd.VS = { vs->GetBufferPointer(), vs->GetBufferSize() };
        pd.PS = { ps->GetBufferPointer(), ps->GetBufferSize() };
        pd.InputLayout = { layout, 2 };
        pd.RasterizerState.FillMode = fill;
        pd.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
        pd.RasterizerState.DepthClipEnable = TRUE;
        pd.DepthStencilState.DepthEnable = TRUE;
        pd.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
        pd.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_LESS;
        pd.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
        pd.SampleMask = UINT_MAX;
        pd.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        pd.NumRenderTargets = 1;
        pd.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
        pd.DSVFormat = DXGI_FORMAT_D32_FLOAT;
        pd.SampleDesc.Count = 1;
        ComPtr<ID3D12PipelineState> pso;
        g.device->CreateGraphicsPipelineState(&pd, IID_PPV_ARGS(&pso));
        return pso;
    }

    void UpdateWater(float t)
    {
        Vertex* dst = (Vertex*)g.vbMapped;
        const size_t count = g.baseX.size();
        for (size_t i = 0; i < count; ++i)
        {
            Vec3 pos, n;
            GerstnerAt(g.baseX[i], g.baseZ[i], t, &pos, &n);
            Vertex& v = dst[i];
            v.px = pos.x; v.py = pos.y; v.pz = pos.z;
            v.nx = n.x;   v.ny = n.y;   v.nz = n.z;
        }
    }

    bool InitGraphics(HWND hwnd)
    {
        bool debugOn = false;
#if defined(_DEBUG)
        ComPtr<ID3D12Debug> dbg;
        if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&dbg))))
        {
            dbg->EnableDebugLayer();
            debugOn = true;
            std::printf("[gfx] D3D12 debug layer ON.\n");
        }
#endif

        ComPtr<IDXGIFactory4> factory;
        const HRESULT hrFactory = debugOn
            ? CreateDXGIFactory2(DXGI_CREATE_FACTORY_DEBUG, IID_PPV_ARGS(&factory))
            : CreateDXGIFactory1(IID_PPV_ARGS(&factory));
        if (FAILED(hrFactory)) return false;

        ComPtr<IDXGIAdapter1> adapter;
        for (UINT i = 0; factory->EnumAdapters1(i, &adapter) != DXGI_ERROR_NOT_FOUND; ++i)
        {
            DXGI_ADAPTER_DESC1 desc{};
            adapter->GetDesc1(&desc);
            if (desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) { adapter.Reset(); continue; }
            std::printf("[gfx] Adapter: %ls\n", desc.Description);
            break;
        }

        if (FAILED(D3D12CreateDevice(adapter.Get(), D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&g.device))))
            return false;

        D3D12_COMMAND_QUEUE_DESC qd{};
        qd.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
        if (FAILED(g.device->CreateCommandQueue(&qd, IID_PPV_ARGS(&g.queue)))) return false;
        if (FAILED(g.device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&g.allocator)))) return false;
        if (FAILED(g.device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, g.allocator.Get(), nullptr, IID_PPV_ARGS(&g.cmdList)))) return false;
        g.cmdList->Close();

        DXGI_SWAP_CHAIN_DESC1 sd{};
        sd.BufferCount = kBackBufferCount;
        sd.Width = kWidth;
        sd.Height = kHeight;
        sd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        sd.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
        sd.SampleDesc.Count = 1;

        ComPtr<IDXGISwapChain1> sc1;
        if (FAILED(factory->CreateSwapChainForHwnd(g.queue.Get(), hwnd, &sd, nullptr, nullptr, &sc1))) return false;
        if (FAILED(sc1.As(&g.swapChain))) return false;
        g.frameIndex = g.swapChain->GetCurrentBackBufferIndex();

        D3D12_DESCRIPTOR_HEAP_DESC hd{};
        hd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
        hd.NumDescriptors = kBackBufferCount;
        if (FAILED(g.device->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&g.rtvHeap)))) return false;
        g.rtvIncrement = g.device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

        for (UINT i = 0; i < kBackBufferCount; ++i)
        {
            ComPtr<ID3D12Resource> bb;
            if (FAILED(g.swapChain->GetBuffer(i, IID_PPV_ARGS(&bb)))) return false;
            D3D12_CPU_DESCRIPTOR_HANDLE rtv = g.rtvHeap->GetCPUDescriptorHandleForHeapStart();
            rtv.ptr += i * g.rtvIncrement;
            g.device->CreateRenderTargetView(bb.Get(), nullptr, rtv);
        }

        D3D12_DESCRIPTOR_HEAP_DESC dd{};
        dd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
        dd.NumDescriptors = 1;
        if (FAILED(g.device->CreateDescriptorHeap(&dd, IID_PPV_ARGS(&g.dsvHeap)))) return false;

        D3D12_RESOURCE_DESC td{};
        td.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        td.Width = kWidth;
        td.Height = kHeight;
        td.DepthOrArraySize = 1;
        td.MipLevels = 1;
        td.Format = DXGI_FORMAT_D32_FLOAT;
        td.SampleDesc.Count = 1;
        td.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;
        D3D12_HEAP_PROPERTIES dhp{};
        dhp.Type = D3D12_HEAP_TYPE_DEFAULT;
        if (FAILED(g.device->CreateCommittedResource(&dhp, D3D12_HEAP_FLAG_NONE, &td,
            D3D12_RESOURCE_STATE_DEPTH_WRITE,
            nullptr, IID_PPV_ARGS(&g.depthTex))))
            return false;
        g.device->CreateDepthStencilView(g.depthTex.Get(), nullptr,
            g.dsvHeap->GetCPUDescriptorHandleForHeapStart());

        // Root signature: 56 root constants (mvpWater | mvpModel | model | light | cam).
        D3D12_ROOT_CONSTANTS rc{};
        rc.Num32BitValues = 56;
        rc.ShaderRegister = 0;
        rc.RegisterSpace = 0;
        D3D12_ROOT_PARAMETER rp{};
        rp.ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
        rp.Constants = rc;
        rp.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
        D3D12_ROOT_SIGNATURE_DESC rsd{};
        rsd.NumParameters = 1;
        rsd.pParameters = &rp;
        rsd.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

        ComPtr<ID3DBlob> sigBlob, sigErr;
        if (FAILED(D3D12SerializeRootSignature(&rsd, D3D_ROOT_SIGNATURE_VERSION_1, &sigBlob, &sigErr)))
        {
            if (sigErr) std::printf("[rs] %s\n", (const char*)sigErr->GetBufferPointer());
            return false;
        }
        if (FAILED(g.device->CreateRootSignature(0, sigBlob->GetBufferPointer(),
            sigBlob->GetBufferSize(), IID_PPV_ARGS(&g.rootSig))))
            return false;

        ComPtr<ID3DBlob> vsWater = CompileShader(L"water_vs.hlsl", "main", "vs_5_0");
        ComPtr<ID3DBlob> vsModel = CompileShader(L"water_vs.hlsl", "mainModel", "vs_5_0");
        ComPtr<ID3DBlob> psWater = CompileShader(L"water_ps.hlsl", "main", "ps_5_0");
        ComPtr<ID3DBlob> psCrate = CompileShader(L"water_ps.hlsl", "psCrate", "ps_5_0");
        if (!vsWater || !vsModel || !psWater || !psCrate) return false;

        g.psoSolid = MakePso(vsWater.Get(), psWater.Get(), D3D12_FILL_MODE_SOLID);
        g.psoWire = MakePso(vsWater.Get(), psWater.Get(), D3D12_FILL_MODE_WIREFRAME);
        g.psoCrate = MakePso(vsModel.Get(), psCrate.Get(), D3D12_FILL_MODE_SOLID);
        if (!g.psoSolid || !g.psoWire || !g.psoCrate) return false;
        std::printf("[gfx] PSOs: water solid/wire + crate. F1 wireframe, R redrop.\n");

        const int n = kCells + 1;
        g.baseX.reserve((size_t)n * n);
        g.baseZ.reserve((size_t)n * n);
        for (int iz = 0; iz < n; ++iz)
            for (int ix = 0; ix < n; ++ix)
            {
                g.baseX.push_back(-kHalf + 2.0f * kHalf * (float)ix / (float)kCells);
                g.baseZ.push_back(-kHalf + 2.0f * kHalf * (float)iz / (float)kCells);
            }

        std::vector<uint16_t> indices;
        indices.reserve((size_t)kCells * kCells * 6);
        for (int iz = 0; iz < kCells; ++iz)
            for (int ix = 0; ix < kCells; ++ix)
            {
                const uint16_t i00 = (uint16_t)(iz * n + ix);
                const uint16_t i10 = (uint16_t)(i00 + 1);
                const uint16_t i01 = (uint16_t)(i00 + n);
                const uint16_t i11 = (uint16_t)(i01 + 1);
                indices.insert(indices.end(), { i00, i01, i10, i10, i01, i11 });
            }

        const UINT64 vbSize = (UINT64)g.baseX.size() * sizeof(Vertex);
        g.vertexBuffer = CreateUploadBuffer(g.device.Get(), nullptr, vbSize);
        g.indexBuffer = CreateUploadBuffer(g.device.Get(), indices.data(),
            (UINT64)indices.size() * sizeof(uint16_t));
        if (!g.vertexBuffer || !g.indexBuffer) return false;
        g.vertexBuffer->Map(0, nullptr, &g.vbMapped);

        g.vbv.BufferLocation = g.vertexBuffer->GetGPUVirtualAddress();
        g.vbv.StrideInBytes = sizeof(Vertex);
        g.vbv.SizeInBytes = (UINT)vbSize;
        g.ibv.BufferLocation = g.indexBuffer->GetGPUVirtualAddress();
        g.ibv.Format = DXGI_FORMAT_R16_UINT;
        g.ibv.SizeInBytes = (UINT)indices.size() * sizeof(uint16_t);
        g.indexCount = (UINT)indices.size();

        // --- Crate mesh: 24 verts (4 per face, real normals), 36 indices. ---
        const float ex = g_body.halfW, ey = g_body.halfH, ez = g_body.halfD;
        struct Face { Vec3 n; Vec3 u; Vec3 v; };
        const Face faces[6] = {
            { { 0, 0,-1 }, { 1, 0, 0 }, { 0, 1, 0 } },
            { { 0, 0, 1 }, { 1, 0, 0 }, { 0, 1, 0 } },
            { {-1, 0, 0 }, { 0, 0, 1 }, { 0, 1, 0 } },
            { { 1, 0, 0 }, { 0, 0, 1 }, { 0, 1, 0 } },
            { { 0,-1, 0 }, { 1, 0, 0 }, { 0, 0, 1 } },
            { { 0, 1, 0 }, { 1, 0, 0 }, { 0, 0, 1 } },
        };
        std::vector<Vertex> cverts;
        std::vector<uint16_t> cindices;
        for (int f = 0; f < 6; ++f)
        {
            const Vec3 n = faces[f].n, u = faces[f].u, v = faces[f].v;
            const Vec3 c = n;   // face center = normal * half-extent along n
            const Vec3 center = { c.x * ex, c.y * ey, c.z * ez };
            const Vec3 eu = { u.x * ex, u.y * ey, u.z * ez };
            const Vec3 ev = { v.x * ex, v.y * ey, v.z * ez };
            const uint16_t base = (uint16_t)cverts.size();
            const float su[4] = { -1, -1, 1, 1 };
            const float sv[4] = { -1, 1, 1, -1 };
            for (int k = 0; k < 4; ++k)
            {
                Vertex vtx{};
                vtx.px = center.x + eu.x * su[k] + ev.x * sv[k];
                vtx.py = center.y + eu.y * su[k] + ev.y * sv[k];
                vtx.pz = center.z + eu.z * su[k] + ev.z * sv[k];
                vtx.nx = n.x; vtx.ny = n.y; vtx.nz = n.z;
                cverts.push_back(vtx);
            }
            const uint16_t b0 = base;
            const uint16_t b1 = (uint16_t)(base + 1);
            const uint16_t b2 = (uint16_t)(base + 2);
            const uint16_t b3 = (uint16_t)(base + 3);
            cindices.insert(cindices.end(), { b0, b1, b2, b0, b2, b3 });
        }
        g.crateVB = CreateUploadBuffer(g.device.Get(), cverts.data(),
            (UINT64)cverts.size() * sizeof(Vertex));
        g.crateIB = CreateUploadBuffer(g.device.Get(), cindices.data(),
            (UINT64)cindices.size() * sizeof(uint16_t));
        if (!g.crateVB || !g.crateIB) return false;
        g.cvbv.BufferLocation = g.crateVB->GetGPUVirtualAddress();
        g.cvbv.StrideInBytes = sizeof(Vertex);
        g.cvbv.SizeInBytes = (UINT)(cverts.size() * sizeof(Vertex));
        g.cibv.BufferLocation = g.crateIB->GetGPUVirtualAddress();
        g.cibv.Format = DXGI_FORMAT_R16_UINT;
        g.cibv.SizeInBytes = (UINT)(cindices.size() * sizeof(uint16_t));

        Mat4LookAtLH({ 0.0f, 2.5f, -9.0f }, { 0.6f, 0.0f, 2.0f }, { 0.0f, 1.0f, 0.0f }, g.view);
        Mat4PerspectiveLH(1.05f, (float)kWidth / (float)kHeight, 0.1f, 200.0f, g.proj);

        if (FAILED(g.device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&g.fence)))) return false;
        g.fenceEvent = CreateEventW(nullptr, FALSE, FALSE, nullptr);
        std::printf("[gfx] Grid: %zu verts, %u indices; crate: %zu verts.\n",
            g.baseX.size(), g.indexCount, cverts.size());
        return true;
    }

    void RenderFrame(float t)
    {
        UpdateWater(t);

        float mvpWater[16], model[16], mvpModel[16], tmp[16];
        Mat4Multiply(g.view, g.proj, mvpWater);
        Mat4FromQuatPos(g_body.q, g_body.pos, model);
        Mat4Multiply(model, g.view, tmp);
        Mat4Multiply(tmp, g.proj, mvpModel);

        float* rc = g.rootConstants;
        for (int i = 0; i < 16; ++i) rc[i] = mvpWater[i];
        for (int i = 0; i < 16; ++i) rc[16 + i] = mvpModel[i];
        for (int i = 0; i < 16; ++i) rc[32 + i] = model[i];
        const Vec3 L = Normalize({ 0.35f, 0.65f, -0.50f });
        rc[48] = L.x; rc[49] = L.y; rc[50] = L.z; rc[51] = 0.0f;
        rc[52] = 0.0f; rc[53] = 2.5f; rc[54] = -9.0f; rc[55] = t;

        g.allocator->Reset();
        g.cmdList->Reset(g.allocator.Get(), nullptr);

        ComPtr<ID3D12Resource> bb;
        g.swapChain->GetBuffer(g.frameIndex, IID_PPV_ARGS(&bb));

        D3D12_RESOURCE_BARRIER barrier{};
        barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        barrier.Transition.pResource = bb.Get();
        barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_PRESENT;
        barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;
        g.cmdList->ResourceBarrier(1, &barrier);

        D3D12_CPU_DESCRIPTOR_HANDLE rtv = g.rtvHeap->GetCPUDescriptorHandleForHeapStart();
        rtv.ptr += g.frameIndex * g.rtvIncrement;
        D3D12_CPU_DESCRIPTOR_HANDLE dsv = g.dsvHeap->GetCPUDescriptorHandleForHeapStart();

        const float clearColor[4] = { 0.01f, 0.03f, 0.06f, 1.0f };
        g.cmdList->ClearRenderTargetView(rtv, clearColor, 0, nullptr);
        g.cmdList->ClearDepthStencilView(dsv, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);

        g.cmdList->OMSetRenderTargets(1, &rtv, FALSE, &dsv);
        g.cmdList->SetGraphicsRootSignature(g.rootSig.Get());
        g.cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

        D3D12_VIEWPORT vp{};
        vp.Width = (float)kWidth;
        vp.Height = (float)kHeight;
        vp.MaxDepth = 1.0f;
        D3D12_RECT scissor{ 0, 0, (LONG)kWidth, (LONG)kHeight };
        g.cmdList->RSSetViewports(1, &vp);
        g.cmdList->RSSetScissorRects(1, &scissor);
        g.cmdList->SetGraphicsRoot32BitConstants(0, 56, rc, 0);

        // Water pass.
        g.cmdList->SetPipelineState(g.wireframe ? g.psoWire.Get() : g.psoSolid.Get());
        g.cmdList->IASetVertexBuffers(0, 1, &g.vbv);
        g.cmdList->IASetIndexBuffer(&g.ibv);
        g.cmdList->DrawIndexedInstanced(g.indexCount, 1, 0, 0, 0);

        // Crate pass.
        g.cmdList->SetPipelineState(g.psoCrate.Get());
        g.cmdList->IASetVertexBuffers(0, 1, &g.cvbv);
        g.cmdList->IASetIndexBuffer(&g.cibv);
        g.cmdList->DrawIndexedInstanced(36, 1, 0, 0, 0);

        barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
        barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_PRESENT;
        g.cmdList->ResourceBarrier(1, &barrier);
        g.cmdList->Close();

        ID3D12CommandList* lists[] = { g.cmdList.Get() };
        g.queue->ExecuteCommandLists(1, lists);
        g.swapChain->Present(1, 0);

        ++g.fenceValue;
        g.queue->Signal(g.fence.Get(), g.fenceValue);
        if (g.fence->GetCompletedValue() < g.fenceValue)
        {
            g.fence->SetEventOnCompletion(g.fenceValue, g.fenceEvent);
            WaitForSingleObject(g.fenceEvent, INFINITE);
        }
        g.frameIndex = g.swapChain->GetCurrentBackBufferIndex();
    }

    void ShutdownGraphics()
    {
        ++g.fenceValue;
        g.queue->Signal(g.fence.Get(), g.fenceValue);
        if (g.fence->GetCompletedValue() < g.fenceValue)
        {
            g.fence->SetEventOnCompletion(g.fenceValue, g.fenceEvent);
            WaitForSingleObject(g.fenceEvent, INFINITE);
        }
        if (g.vbMapped) g.vertexBuffer->Unmap(0, nullptr);
        CloseHandle(g.fenceEvent);
        std::printf("[gfx] GPU idle, shutdown clean.\n");
    }

} // namespace

int main()
{
    std::printf("CFD_but_actually_fun: opening window...\n");
    InitWaves();
    InitBody();

    WNDCLASSEXW wc{};
    wc.cbSize = sizeof(wc);
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = GetModuleHandleW(nullptr);
    wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
    wc.lpszClassName = L"CFDWaterWindow";
    if (!RegisterClassExW(&wc)) return 1;

    HWND hwnd = CreateWindowExW(0, wc.lpszClassName, L"CFD but actually fun",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT, kWidth, kHeight,
        nullptr, nullptr, wc.hInstance, nullptr);
    if (!hwnd) return 1;
    ShowWindow(hwnd, SW_SHOW);
    UpdateWindow(hwnd);

    if (!InitGraphics(hwnd))
    {
        std::printf("[gfx] Initialization FAILED.\n");
        return 1;
    }

    double prev = NowSeconds();
    bool running = true;
    while (running)
    {
        MSG msg{};
        while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE))
        {
            if (msg.message == WM_QUIT) running = false;
            else { TranslateMessage(&msg); DispatchMessageW(&msg); }
        }
        if (!running) break;

        const bool f1Down = (GetAsyncKeyState(VK_F1) & 0x8000) != 0;
        if (f1Down && !g.prevF1)
        {
            g.wireframe = !g.wireframe;
            std::printf("[ui] wireframe %s\n", g.wireframe ? "ON" : "OFF");
        }
        g.prevF1 = f1Down;

        const bool rDown = (GetAsyncKeyState('R') & 0x8000) != 0;
        if (rDown && !g.prevR) DropBody();
        g.prevR = rDown;

        // Fixed-timestep physics: frame-rate independent, deterministic.
        const double now = NowSeconds();
        double frameDt = now - prev;
        prev = now;
        if (frameDt > 0.25) frameDt = 0.25;
        g_physAcc += (float)frameDt;
        while (g_physAcc >= kSimDt)
        {
            StepBody(kSimDt);
            g_simTime += kSimDt;
            g_physAcc -= kSimDt;
        }

        RenderFrame(g_simTime);
    }

    ShutdownGraphics();
    std::printf("Window closed cleanly. Goodbye.\n");
    return 0;
}
