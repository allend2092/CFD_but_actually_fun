#define NOMINMAX
#include <windows.h>

// main.cpp
// Milestone 5: Jetski integration. The local model's rider meets the Gerstner ocean.

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
#include <span>

#include "assets/jetski_asset.h"
#include "assets/jetski_internal.h"   // complete definition of jetski::Rig
#include "assets/boat_sim.h"          // real-hull boat rigid body (planing + trim)
#include "assets/wake_field.h"        // interactive boat-locked wake heightfield

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
    constexpr float kHalf = 20.0f;   // radius of the boat-locked water patch (m)
    constexpr float kGravity = 9.81f;
    constexpr float kSimDt = 1.0f / 120.0f;
    constexpr float kSteepness = 0.80f;

    struct Vertex { float px, py, pz, nx, ny, nz; };

    struct WaveRecipe { float dirX, dirZ, wavelength, amp, phase0; };
    constexpr WaveRecipe kWaveRecipes[] = {
     {  1.00f, 0.15f, 14.0f, 0.55f, 0.0f },  // Long, tall primary swell (highly visible)
     {  0.80f, 0.60f,  8.0f, 0.30f, 1.7f },  // Medium cross-chop
     {  0.35f, 1.00f,  4.5f, 0.18f, 3.1f },  // Shorter wind waves
     { -0.25f, 0.95f,  2.5f, 0.10f, 4.2f },  // Small detail ripples
    };
    constexpr int kNumWaves = 4;

    struct WaveRuntime { float dirX, dirZ, k, omega, amp, phase0, q; };
    WaveRuntime g_waves[kNumWaves];

    float g_throttle = 0.0f;
    float g_steer = 0.0f;

    // Rider model: true = static rider (rigid copy of the rest pose attached to
    // the hull — safe at any heading); false = full IK physics. P key toggles.
    // Default STATIC until the IK's hull roll/pitch extraction is fixed for
    // large heading changes (it degenerates near +/-180 deg yaw).
    bool g_riderStatic = true;

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

    // ---------------------------------------------------------------- camera
    struct ChaseCamera {
        Vec3 pos;
        Vec3 vel;
        Vec3 lookAt;
        Vec3 lookAtVel;

        float dist = 5.5f;       // Distance behind the hull
        float height = 2.2f;     // Height above the hull
        float lookAhead = 8.0f;  // Distance ahead to look (anticipates turns)

        float posStiffness = 6.0f;
        float posDamping = 4.5f;
        float lookStiffness = 10.0f;
        float lookDamping = 6.0f;

        float shakeTime = 0.0f;
        Vec3 fwd = { 0.0f, 0.0f, 1.0f };  // smoothed hull forward (yaw lag)
    };
    ChaseCamera g_cam;
    float g_viewMat[16] = {};   // view matrix, written by UpdateCamera every frame



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
            w.omega = std::sqrt(kGravity * w.k);
            w.amp = r.amp;
            w.phase0 = r.phase0;
            w.q = kSteepness / (w.k * w.amp * (float)kNumWaves);
        }
    }

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
    boat::Boat      g_boat;
    boat::WakeField g_wake(26.0f, 128, 2.6f, 1.4f, 0.9f);   // S,N,c,mu,sponge
    float g_simTime = 0.0f;
    float g_physAcc = 0.0f;
    float g_printAcc = 0.0f;

    void InitCamera();
    void UpdateCamera(float dt, float t);

    void ResetBoat()
    {
        g_boat.reset(0.0f, 0.8f, 0.0f);
        g_wake.clear();
        InitCamera();
    }

  void StepBoat(float dt)
    {
        const boat::Vec3 oldPos = g_boat.pos();

        // 1) water height at each hull sample = Gerstner + boat-locked wake
        float wh[8];
        for (int i = 0; i < g_boat.nSamples(); ++i)
        {
            const boat::Vec3 sw = g_boat.sampleWorld(i);
            float h; Vec3 n;
            SampleWaterWorld(sw.x, sw.z, g_simTime, &h, &n);
            wh[i] = h + g_wake.sample(sw.x - oldPos.x, sw.z - oldPos.z);
        }

        // 2) advance the boat (real hull + planing + trim), guarded
        g_boat.setControls(g_throttle, g_steer);
        bool whOk = true;
        for (int i = 0; i < g_boat.nSamples(); ++i) if (!std::isfinite(wh[i])) whOk = false;
        if (whOk) g_boat.step(dt, wh);
        // Watchdog: self-heal instead of black-screening if the boat ever diverges
        {
            const boat::Vec3 pp = g_boat.pos();
            const boat::Vec3 vv = g_boat.vel();
            const float fin = pp.x + pp.y + pp.z + vv.x + vv.y + vv.z;
            if (!std::isfinite(fin) || pp.y < -15.0f || pp.y > 40.0f)
            {
                std::printf("[watchdog] boat diverged (y=%.2f); resetting to center\n", pp.y);
                g_boat.reset(0.0f, 0.8f, 0.0f);
                g_wake.clear();
            }
        }

        // 3) boat-locked scroll so the wake trails the boat
        const boat::Vec3 dp = boat::vsub(g_boat.pos(), oldPos);
        g_wake.scroll(dp.x, dp.z);

        // 4) inject: the hull plows the surface. Proximity-gated so a planing
        //    (skimming) hull still writes its wake. Field coord = boat-relative.
        const boat::Vec3 v    = g_boat.vel();
        const float vY  = v.y;
        const float vFwd = std::sqrt(v.x * v.x + v.z * v.z);
        for (int i = 0; i < g_boat.nSamples(); ++i)
        {
            const boat::Vec3 sw = g_boat.sampleWorld(i);
            const float rx = sw.x - g_boat.pos().x, rz = sw.z - g_boat.pos().z;
            float h; Vec3 n;
            SampleWaterWorld(sw.x, sw.z, g_simTime, &h, &n);
            const float hSurf = h + g_wake.sample(rx, rz);
            const float above = sw.y - hSurf;                       // >0 above water
            const float prox  = std::clamp(1.0f - std::max(0.0f, above) / 0.3f, 0.0f, 1.0f);
            const float wgt   = g_boat.config().samples[i].w * prox;
            g_wake.inject(rx, rz, 0.0f, (vY * 4.0f - vFwd * 3.0f) * wgt * dt);
        }

        // 5) advance the wake field
        g_wake.step(dt);
    }

  void InitCamera()
  {
      jetski::VehicleState vs{};
      g_boat.vehicleState(0.0f, 0.0f, boat::Vec3{ 0, 1, 0 }, vs);

      Vec3 hullX = { vs.hullBasis[0], vs.hullBasis[3], vs.hullBasis[6] };
      Vec3 hullY = { vs.hullBasis[1], vs.hullBasis[4], vs.hullBasis[7] };
      Vec3 hullZ = { vs.hullBasis[2], vs.hullBasis[5], vs.hullBasis[8] };
      Vec3 hullPos = { vs.hullPosition[0], vs.hullPosition[1], vs.hullPosition[2] };

      // Snap camera to ideal position on init so it doesn't fly across the ocean on frame 1
      g_cam.pos = hullPos - hullZ * g_cam.dist + hullY * g_cam.height;
      g_cam.vel = { 0, 0, 0 };
      g_cam.lookAt = hullPos + hullZ * g_cam.lookAhead;
      g_cam.lookAtVel = { 0, 0, 0 };
      g_cam.fwd = hullZ;
  }

  void UpdateCamera(float dt, float t)
  {
      jetski::VehicleState vs{};
      g_boat.vehicleState(t, 0.0f, boat::Vec3{ 0, 1, 0 }, vs);

      Vec3 hullX = { vs.hullBasis[0], vs.hullBasis[3], vs.hullBasis[6] };
      Vec3 hullY = { vs.hullBasis[1], vs.hullBasis[4], vs.hullBasis[7] };
      Vec3 hullZ = { vs.hullBasis[2], vs.hullBasis[5], vs.hullBasis[8] };
      Vec3 hullPos = { vs.hullPosition[0], vs.hullPosition[1], vs.hullPosition[2] };

      // A spring-damper tracking a moving target lags by v*(damping/stiffness).
      // Feed the boat velocity forward to cancel it exactly at steady speed;
      // the spring still smooths turns and wave impacts.
      const Vec3 boatVel = { vs.hullVelocity[0], vs.hullVelocity[1], vs.hullVelocity[2] };
      // Camera yaw chases hull yaw with a ~0.3 s lag: during turns the hull
      // angles visibly across frame instead of the whole world rotating invisibly.
      const float kF = 1.0f - std::exp(-dt / 0.30f);
      g_cam.fwd = Normalize(g_cam.fwd + (hullZ - g_cam.fwd) * kF);
      Vec3 idealPos = hullPos - g_cam.fwd * g_cam.dist + hullY * g_cam.height + boatVel * (g_cam.posDamping / g_cam.posStiffness);
      Vec3 idealLook = hullPos + g_cam.fwd * g_cam.lookAhead + hullY * 0.5f + boatVel * (g_cam.lookDamping / g_cam.lookStiffness);


      // 2. Spring-damper for position (smooth follow)
      Vec3 posDiff = idealPos - g_cam.pos;
      Vec3 posAccel = posDiff * g_cam.posStiffness - g_cam.vel * g_cam.posDamping;
      g_cam.vel = g_cam.vel + posAccel * dt;
      g_cam.pos = g_cam.pos + g_cam.vel * dt;

      // 3. Spring-damper for look-at (smooth anticipation)
      Vec3 lookDiff = idealLook - g_cam.lookAt;
      Vec3 lookAccel = lookDiff * g_cam.lookStiffness - g_cam.lookAtVel * g_cam.lookDamping;
      g_cam.lookAtVel = g_cam.lookAtVel + lookAccel * dt;
      g_cam.lookAt = g_cam.lookAt + g_cam.lookAtVel * dt;

      // 4. Camera shake based on wave chop and speed
      float hGerstner; Vec3 nGerstner;
      SampleWaterWorld(hullPos.x, hullPos.z, t, &hGerstner, &nGerstner);
      float hWake = g_wake.sample(0.0f, 0.0f); // 0,0 in boat-locked space is hull center
      float surfaceY = hGerstner + hWake;

      float submersion = surfaceY - hullPos.y;
      float speed = g_boat.speed();
      float chop = std::max(0.0f, submersion) * 0.8f + std::max(0.0f, speed - 5.0f) * 0.05f;
      chop = std::min(chop, 1.0f);

      float freq = 12.0f + speed * 1.5f;
      g_cam.shakeTime += dt * freq;

      float sx = std::sin(g_cam.shakeTime * 1.3f) * std::cos(g_cam.shakeTime * 0.7f + 1.0f);
      float sy = std::sin(g_cam.shakeTime * 1.7f + 2.0f) * std::cos(g_cam.shakeTime * 1.1f);
      float sz = std::sin(g_cam.shakeTime * 0.9f + 3.0f) * std::cos(g_cam.shakeTime * 1.5f);

      float shakeScale = chop * 0.12f;
      Vec3 shakeOffset = { sx * shakeScale, sy * shakeScale * 0.5f, sz * shakeScale };

      Vec3 finalPos = g_cam.pos + shakeOffset;
      Vec3 finalLook = g_cam.lookAt;

      Mat4LookAtLH(finalPos, finalLook, Vec3{ 0.0f, 1.0f, 0.0f }, g_viewMat);
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
        ComPtr<ID3D12PipelineState>       psoJetski;
        ComPtr<ID3D12Resource>            vertexBuffer;
        ComPtr<ID3D12Resource>            indexBuffer;
        ComPtr<ID3D12Resource>            crateVB;
        ComPtr<ID3D12Resource>            crateIB;
        ComPtr<ID3D12Resource>            jetskiVB;
        ComPtr<ID3D12Resource>            jetskiIB;
        ComPtr<ID3D12Fence>               fence;
        D3D12_VERTEX_BUFFER_VIEW          vbv{};
        D3D12_INDEX_BUFFER_VIEW           ibv{};
        D3D12_VERTEX_BUFFER_VIEW          cvbv{};
        D3D12_INDEX_BUFFER_VIEW           cibv{};
        D3D12_VERTEX_BUFFER_VIEW          jvbv{};
        D3D12_INDEX_BUFFER_VIEW           jibv{};
        HANDLE fenceEvent = nullptr;
        void* vbMapped = nullptr;
        void* jvbMapped = nullptr;
        UINT64 fenceValue = 0;
        UINT   rtvIncrement = 0;
        UINT   frameIndex = 0;
        UINT   indexCount = 0;
        bool   wireframe = false;
        bool   prevF1 = false;
        bool   prevR = false;
        bool   prevP = false;
        float  proj[16] = {};
        float  rootConstants[64] = {};
        std::vector<float> baseX, baseZ;


        jetski::Rig                       rig;
        std::vector<jetski::AssetVertex>  posedVerts;
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
        const boat::Vec3 bp = g_boat.pos();
        for (size_t i = 0; i < count; ++i)
        {
            // The GRID is boat-locked (a moving window), but the WAVES are sampled
            // in WORLD space: the ocean stays put while the window slides over it.
            const float wx = bp.x + g.baseX[i];
            const float wz = bp.z + g.baseZ[i];
            Vec3 pos, n;
            GerstnerAt(wx, wz, t, &pos, &n);
            pos.y += g_wake.sample(g.baseX[i], g.baseZ[i]);   // wake field IS boat-local
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

        D3D12_ROOT_CONSTANTS rc{};
        rc.Num32BitValues = 64;
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
            return false;
        if (FAILED(g.device->CreateRootSignature(0, sigBlob->GetBufferPointer(),
            sigBlob->GetBufferSize(), IID_PPV_ARGS(&g.rootSig))))
            return false;

        ComPtr<ID3DBlob> vsWater = CompileShader(L"water_vs.hlsl", "main", "vs_5_0");
        ComPtr<ID3DBlob> vsModel = CompileShader(L"water_vs.hlsl", "mainModel", "vs_5_0");
        ComPtr<ID3DBlob> vsJetski = CompileShader(L"water_vs.hlsl", "mainJetski", "vs_5_0");
        ComPtr<ID3DBlob> psWater = CompileShader(L"water_ps.hlsl", "main", "ps_5_0");
        ComPtr<ID3DBlob> psCrate = CompileShader(L"water_ps.hlsl", "psCrate", "ps_5_0");
        ComPtr<ID3DBlob> psJetski = CompileShader(L"water_ps.hlsl", "psJetski", "ps_5_0");
        if (!vsWater || !vsModel || !vsJetski || !psWater || !psCrate || !psJetski) return false;

        g.psoSolid = MakePso(vsWater.Get(), psWater.Get(), D3D12_FILL_MODE_SOLID);
        g.psoWire = MakePso(vsWater.Get(), psWater.Get(), D3D12_FILL_MODE_WIREFRAME);
        g.psoCrate = MakePso(vsModel.Get(), psCrate.Get(), D3D12_FILL_MODE_SOLID);

        D3D12_INPUT_ELEMENT_DESC layoutJetski[] = {
            { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0,  D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
            { "NORMAL",   0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
            { "COLOR",    0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        };
        D3D12_GRAPHICS_PIPELINE_STATE_DESC pdJ{};
        pdJ.pRootSignature = g.rootSig.Get();
        pdJ.VS = { vsJetski->GetBufferPointer(), vsJetski->GetBufferSize() };
        pdJ.PS = { psJetski->GetBufferPointer(), psJetski->GetBufferSize() };
        pdJ.InputLayout = { layoutJetski, 3 };
        pdJ.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
        pdJ.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
        pdJ.RasterizerState.DepthClipEnable = TRUE;
        pdJ.DepthStencilState.DepthEnable = TRUE;
        pdJ.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
        pdJ.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_LESS;
        pdJ.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
        pdJ.SampleMask = UINT_MAX;
        pdJ.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        pdJ.NumRenderTargets = 1;
        pdJ.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
        pdJ.DSVFormat = DXGI_FORMAT_D32_FLOAT;
        pdJ.SampleDesc.Count = 1;
        if (FAILED(g.device->CreateGraphicsPipelineState(&pdJ, IID_PPV_ARGS(&g.psoJetski)))) return false;

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

        const float ex = 0.45f, ey = 0.35f, ez = 0.45f;   // crate half-extents (was the old boat box size)
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
            const Vec3 c = n;
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

            // The narrowing fix: explicitly cast to uint16_t to satisfy MSVC
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

        // Jetski setup
        jetski::rig_create(g.rig);
        const auto& topo = jetski::rig_topology(g.rig);
        g.posedVerts.resize(topo.vertexCount);

        const UINT64 jvbSize = (UINT64)topo.vertexCount * sizeof(jetski::AssetVertex);
        g.jetskiVB = CreateUploadBuffer(g.device.Get(), nullptr, jvbSize);
        g.jetskiVB->Map(0, nullptr, &g.jvbMapped);
        g.jvbv.BufferLocation = g.jetskiVB->GetGPUVirtualAddress();
        g.jvbv.StrideInBytes = sizeof(jetski::AssetVertex);
        g.jvbv.SizeInBytes = (UINT)jvbSize;

        const UINT64 jibSize = (UINT64)topo.indexCount * sizeof(uint32_t);
        g.jetskiIB = CreateUploadBuffer(g.device.Get(), topo.indices, jibSize);
        g.jibv.BufferLocation = g.jetskiIB->GetGPUVirtualAddress();
        g.jibv.Format = DXGI_FORMAT_R32_UINT;
        g.jibv.SizeInBytes = (UINT)jibSize;

        InitCamera();
        Mat4PerspectiveLH(1.05f, (float)kWidth / (float)kHeight, 0.1f, 200.0f, g.proj);

        if (FAILED(g.device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&g.fence)))) return false;
        g.fenceEvent = CreateEventW(nullptr, FALSE, FALSE, nullptr);
        return true;
    }

    void RenderFrame(float t, float dt)
    {
        UpdateWater(t);        // write this frame's wave heights into the water VB
        UpdateCamera(dt, t);

        float mvpWater[16], model[16], mvpModel[16], tmp[16];
        Mat4Multiply(g_viewMat, g.proj, mvpWater);

        // Fill the rider state from the boat (real hull). Support height/normal
        // under the hull center = Gerstner + wake (field coord 0 = boat center).
        jetski::VehicleState vs{};
        {
            float supH; Vec3 supN;
            SampleWaterWorld(g_boat.pos().x, g_boat.pos().z, t, &supH, &supN);
            supH += g_wake.sample(0, 0);
            g_boat.vehicleState(t, supH, boat::Vec3{supN.x, supN.y, supN.z}, vs);
        }
        // World model matrix from the boat's basis (cols X,Y,Z) + position.
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                model[i * 4 + j] = vs.hullBasis[j * 3 + i];   // transpose: axes into rows
        model[3] = 0; model[7] = 0; model[11] = 0;
        model[12] = vs.hullPosition[0]; model[13] = vs.hullPosition[1]; model[14] = vs.hullPosition[2];
        model[15] = 1;

        if (g_riderStatic)
            jetski::rig_pose_static(g.rig, vs, std::span<jetski::AssetVertex>(g.posedVerts));
        else
            jetski::rig_pose(g.rig, vs, std::span<jetski::AssetVertex>(g.posedVerts));
        memcpy(g.jvbMapped, g.posedVerts.data(), g.posedVerts.size() * sizeof(jetski::AssetVertex));

        Mat4Multiply(model, g_viewMat, tmp);
        Mat4Multiply(tmp, g.proj, mvpModel);

        float* rc = g.rootConstants;
        for (int i = 0; i < 16; ++i) rc[i] = mvpWater[i];
        // Crate = static buoy at world origin: view*proj with an identity model.
        for (int i = 0; i < 16; ++i) rc[16 + i] = mvpWater[i];
        const float ident[16] = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
        for (int i = 0; i < 16; ++i) rc[32 + i] = ident[i];
        const Vec3 L = Normalize({ 0.35f, 0.65f, -0.50f });
        rc[48] = L.x; rc[49] = L.y; rc[50] = L.z; rc[51] = 0.0f;
        rc[52] = g_cam.pos.x; rc[53] = g_cam.pos.y; rc[54] = g_cam.pos.z; rc[55] = t;
        // boatState + wakeParams for the water foam (procedural V in water_ps).
        const float fl = std::sqrt(vs.hullBasis[2] * vs.hullBasis[2] + vs.hullBasis[8] * vs.hullBasis[8]);
        rc[56] = vs.hullPosition[0]; rc[57] = vs.hullPosition[2];
        rc[58] = (fl > 1e-5f) ? vs.hullBasis[2] / fl : 0.0f;   // fwd.x = hull Z axis, x
        rc[59] = (fl > 1e-5f) ? vs.hullBasis[8] / fl : 1.0f;   // fwd.z = hull Z axis, z
        rc[60] = g_boat.speed(); rc[61] = g_throttle; rc[62] = 1.0f; rc[63] = t;

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

        const float clearColor[4] = { 0.04f, 0.09f, 0.13f, 1.0f };
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
        g.cmdList->SetGraphicsRoot32BitConstants(0, 64, rc, 0);

        g.cmdList->SetPipelineState(g.wireframe ? g.psoWire.Get() : g.psoSolid.Get());
        g.cmdList->IASetVertexBuffers(0, 1, &g.vbv);
        g.cmdList->IASetIndexBuffer(&g.ibv);
        g.cmdList->DrawIndexedInstanced(g.indexCount, 1, 0, 0, 0);

        g.cmdList->SetPipelineState(g.psoCrate.Get());
        g.cmdList->IASetVertexBuffers(0, 1, &g.cvbv);
        g.cmdList->IASetIndexBuffer(&g.cibv);
        g.cmdList->DrawIndexedInstanced(36, 1, 0, 0, 0);

        g.cmdList->SetPipelineState(g.psoJetski.Get());
        g.cmdList->IASetVertexBuffers(0, 1, &g.jvbv);
        g.cmdList->IASetIndexBuffer(&g.jibv);
        g.cmdList->DrawIndexedInstanced((UINT)jetski::rig_topology(g.rig).indexCount, 1, 0, 0, 0);

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
        if (g.jvbMapped) g.jetskiVB->Unmap(0, nullptr);
        jetski::rig_destroy(g.rig);
        CloseHandle(g.fenceEvent);
    }

} // namespace

int main()
{
    InitWaves();
    ResetBoat();

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

    if (!InitGraphics(hwnd)) return 1;
    std::printf("Controls: W/S throttle, A/D steer, R reset to center, F1 wireframe, P rider physics (IK on/off).\n");

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
        if (f1Down && !g.prevF1) g.wireframe = !g.wireframe;
        g.prevF1 = f1Down;

        const bool rDown = (GetAsyncKeyState('R') & 0x8000) != 0;
        if (rDown && !g.prevR) ResetBoat();
        g.prevR = rDown;

        // P = rider physics: toggles between the static rider (rigid on the
        // hull, default) and the full IK rig.
        const bool pDown = (GetAsyncKeyState('P') & 0x8000) != 0;
        if (pDown && !g.prevP) g_riderStatic = !g_riderStatic;
        g.prevP = pDown;

        const double now = NowSeconds();
        double frameDt = now - prev;
        prev = now;
        if (frameDt > 0.25) frameDt = 0.25;

        const float dtInput = (float)frameDt;
        if (GetAsyncKeyState('W') & 0x8000) g_throttle = std::min(1.0f, g_throttle + 2.0f * dtInput);
        else g_throttle = std::max(0.0f, g_throttle - 2.0f * dtInput);

        if (GetAsyncKeyState('A') & 0x8000) g_steer = std::max(-1.0f, g_steer - 3.0f * dtInput);
        else if (GetAsyncKeyState('D') & 0x8000) g_steer = std::min(1.0f, g_steer + 3.0f * dtInput);
        else g_steer *= std::exp(-5.0f * dtInput);

        g_physAcc += (float)frameDt;
        while (g_physAcc >= kSimDt)
        {
            StepBoat(kSimDt);
            g_simTime += kSimDt;
            g_physAcc -= kSimDt;
        }

        RenderFrame(g_simTime, (float)frameDt);
        g_printAcc += (float)frameDt;
        if (g_printAcc >= 2.0f)
        {
            g_printAcc = 0.0f;
            const boat::Vec3 pp = g_boat.pos();

            // Calculate heading from the hull's forward vector (Z axis)
            jetski::VehicleState vsT{};
            g_boat.vehicleState(g_simTime, 0.0f, boat::Vec3{ 0, 1, 0 }, vsT);
            const float hdg = std::atan2(vsT.hullBasis[2], vsT.hullBasis[8]) * 57.29578f;

            std::printf("[telemetry] speed %5.1f m/s  hdg %+6.1f deg  pos (%6.1f, %6.1f)  trim %+5.1f deg  roll %+5.1f deg  rider %s\n",
                g_boat.speed(), hdg, pp.x, pp.z,
                g_boat.trim() * 57.29578f, g_boat.roll() * 57.29578f,
                g_riderStatic ? "STATIC" : "IK");
        }
    }

    ShutdownGraphics();
    return 0;
}