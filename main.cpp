// main.cpp
// Milestone 3: animated sum-of-sines water with deep-water dispersion,
// analytic normals, solid lighting, crest foam, F1 wireframe toggle.

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

    constexpr int   kCells = 128;         // Finer grid: resolves short waves.
    constexpr float kHalf = 10.0f;
    constexpr float kGravity = 9.81f;

    struct Vertex { float px, py, pz, nx, ny, nz; };

    // Wave recipe: direction, wavelength, amplitude, initial phase.
    struct WaveRecipe { float dirX, dirZ, wavelength, amp, phase0; };
    constexpr WaveRecipe kWaveRecipes[] = {
        {  1.00f, 0.15f, 7.0f, 0.32f, 0.0f },   // Long swell.
        {  0.80f, 0.60f, 4.3f, 0.18f, 1.7f },   // Medium chop.
        {  0.35f, 1.00f, 2.6f, 0.10f, 3.1f },   // Short chop.
        { -0.25f, 0.95f, 1.6f, 0.05f, 4.2f },   // Ripples.
    };

    struct WaveRuntime { float dirX, dirZ, k, omega, amp, phase0; };
    std::vector<WaveRuntime> g_waves;

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
        ComPtr<ID3D12Resource>            vertexBuffer;
        ComPtr<ID3D12Resource>            indexBuffer;
        ComPtr<ID3D12Fence>               fence;
        D3D12_VERTEX_BUFFER_VIEW          vbv{};
        D3D12_INDEX_BUFFER_VIEW           ibv{};
        HANDLE fenceEvent = nullptr;
        void* vbMapped = nullptr;
        UINT64 fenceValue = 0;
        UINT   rtvIncrement = 0;
        UINT   frameIndex = 0;
        UINT   indexCount = 0;
        bool   wireframe = false;
        bool   prevF1 = false;
        float  mvp[16] = {};
        float  rootConstants[24] = {};
        std::vector<float> baseX, baseZ;
    } g;

    // ---------------------------------------------------------------- small math
    struct Vec3 { float x, y, z; };
    inline Vec3 operator-(Vec3 a, Vec3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
    inline float Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline Vec3 Cross(Vec3 a, Vec3 b)
    {
        return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }
    inline Vec3 Normalize(Vec3 v) { const float l = std::sqrt(Dot(v, v)); return { v.x / l, v.y / l, v.z / l }; }

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

    double NowSeconds()
    {
        using namespace std::chrono;
        return duration<double>(steady_clock::now().time_since_epoch()).count();
    }

    LRESULT CALLBACK WindowProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
    {
        switch (msg)
        {
        case WM_DESTROY: PostQuitMessage(0); return 0;
        default: return DefWindowProcW(hwnd, msg, wParam, lParam);
        }
    }

    // ---------------------------------------------------------------- water sim
    void InitWaves()
    {
        for (const auto& r : kWaveRecipes)
        {
            const float len = std::sqrt(r.dirX * r.dirX + r.dirZ * r.dirZ);
            WaveRuntime w{};
            w.dirX = r.dirX / len;
            w.dirZ = r.dirZ / len;
            w.k = 6.28318530718f / r.wavelength;          // k = 2*pi / lambda
            w.omega = std::sqrt(kGravity * w.k);          // Deep-water dispersion!
            w.amp = r.amp;
            w.phase0 = r.phase0;
            g_waves.push_back(w);
        }
        std::printf("[water] %zu sine components, dispersion omega=sqrt(g*k).\n", g_waves.size());
    }

    // Height + analytic derivatives -> position + normal, written straight into
    // the persistently-mapped upload buffer.
    void UpdateWater(double t)
    {
        Vertex* dst = (Vertex*)g.vbMapped;
        const size_t count = g.baseX.size();
        const float tf = (float)t;
        for (size_t i = 0; i < count; ++i)
        {
            const float x = g.baseX[i];
            const float z = g.baseZ[i];
            float h = 0.0f, dhx = 0.0f, dhz = 0.0f;
            for (const auto& w : g_waves)
            {
                const float phase = w.k * (w.dirX * x + w.dirZ * z) - w.omega * tf + w.phase0;
                const float s = std::sin(phase);
                const float c = std::cos(phase);
                h += w.amp * s;
                dhx += w.amp * w.k * w.dirX * c;
                dhz += w.amp * w.k * w.dirZ * c;
            }
            const float invLen = 1.0f / std::sqrt(dhx * dhx + 1.0f + dhz * dhz);
            Vertex& v = dst[i];
            v.px = x;  v.py = h;  v.pz = z;
            v.nx = -dhx * invLen;  v.ny = invLen;  v.nz = -dhz * invLen;
        }
    }

    // ---------------------------------------------------------------- D3D helpers
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

        // --- Root signature: 24 root constants (mvp + light + camera/time). ---
        D3D12_ROOT_CONSTANTS rc{};
        rc.Num32BitValues = 24;
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

        ComPtr<ID3DBlob> vs = CompileShader(L"water_vs.hlsl", "main", "vs_5_0");
        ComPtr<ID3DBlob> ps = CompileShader(L"water_ps.hlsl", "main", "ps_5_0");
        if (!vs || !ps) return false;

        g.psoSolid = MakePso(vs.Get(), ps.Get(), D3D12_FILL_MODE_SOLID);
        g.psoWire = MakePso(vs.Get(), ps.Get(), D3D12_FILL_MODE_WIREFRAME);
        if (!g.psoSolid || !g.psoWire) return false;
        std::printf("[gfx] PSOs created (solid + wireframe, F1 toggles).\n");

        // --- Grid topology (static indices; vertices animate every frame). ---
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
        g.vertexBuffer = CreateUploadBuffer(g.device.Get(), nullptr, vbSize);  // No initial data.
        g.indexBuffer = CreateUploadBuffer(g.device.Get(), indices.data(),
            (UINT64)indices.size() * sizeof(uint16_t));
        if (!g.vertexBuffer || !g.indexBuffer) return false;
        g.vertexBuffer->Map(0, nullptr, &g.vbMapped);   // Persistent map: write every frame.

        g.vbv.BufferLocation = g.vertexBuffer->GetGPUVirtualAddress();
        g.vbv.StrideInBytes = sizeof(Vertex);
        g.vbv.SizeInBytes = (UINT)vbSize;
        g.ibv.BufferLocation = g.indexBuffer->GetGPUVirtualAddress();
        g.ibv.Format = DXGI_FORMAT_R16_UINT;
        g.ibv.SizeInBytes = (UINT)indices.size() * sizeof(uint16_t);
        g.indexCount = (UINT)indices.size();
        std::printf("[gfx] Grid: %zu verts, %u indices.\n", g.baseX.size(), g.indexCount);

        float view[16], proj[16];
        Mat4LookAtLH({ 0.0f, 2.5f, -9.0f }, { 0.0f, 0.0f, 2.0f }, { 0.0f, 1.0f, 0.0f }, view);
        Mat4PerspectiveLH(1.05f, (float)kWidth / (float)kHeight, 0.1f, 200.0f, proj);
        Mat4Multiply(view, proj, g.mvp);

        if (FAILED(g.device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&g.fence)))) return false;
        g.fenceEvent = CreateEventW(nullptr, FALSE, FALSE, nullptr);
        return true;
    }

    void RenderFrame(double timeSec)
    {
        UpdateWater(timeSec);   // CPU writes straight into the mapped buffer.

        // Pack root constants: mvp | lightDir | camPos+time.
        for (int i = 0; i < 16; ++i) g.rootConstants[i] = g.mvp[i];
        const Vec3 L = Normalize({ 0.35f, 0.65f, -0.50f });
        g.rootConstants[16] = L.x; g.rootConstants[17] = L.y; g.rootConstants[18] = L.z; g.rootConstants[19] = 0.0f;
        g.rootConstants[20] = 0.0f; g.rootConstants[21] = 2.5f; g.rootConstants[22] = -9.0f;
        g.rootConstants[23] = (float)timeSec;

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
        g.cmdList->SetPipelineState(g.wireframe ? g.psoWire.Get() : g.psoSolid.Get());
        g.cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        g.cmdList->IASetVertexBuffers(0, 1, &g.vbv);
        g.cmdList->IASetIndexBuffer(&g.ibv);

        D3D12_VIEWPORT vp{};
        vp.Width = (float)kWidth;
        vp.Height = (float)kHeight;
        vp.MaxDepth = 1.0f;
        D3D12_RECT scissor{ 0, 0, (LONG)kWidth, (LONG)kHeight };
        g.cmdList->RSSetViewports(1, &vp);
        g.cmdList->RSSetScissorRects(1, &scissor);

        g.cmdList->SetGraphicsRoot32BitConstants(0, 24, g.rootConstants, 0);
        g.cmdList->DrawIndexedInstanced(g.indexCount, 1, 0, 0, 0);

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

    const double start = NowSeconds();
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

        // F1 edge-detect: toggle wireframe debug view.
        const bool f1Down = (GetAsyncKeyState(VK_F1) & 0x8000) != 0;
        if (f1Down && !g.prevF1)
        {
            g.wireframe = !g.wireframe;
            std::printf("[ui] wireframe %s\n", g.wireframe ? "ON" : "OFF");
        }
        g.prevF1 = f1Down;

        RenderFrame(NowSeconds() - start);
    }

    ShutdownGraphics();
    std::printf("Window closed cleanly. Goodbye.\n");
    return 0;
}