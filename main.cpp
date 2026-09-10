// main.cpp
// Milestone 1b: Win32 window + DirectX 12 device + swap chain + animated clear color.

#include <windows.h>
#include <d3d12.h>
#include <dxgi1_6.h>
#include <wrl/client.h>
#include <chrono>
#include <cmath>
#include <cstdio>

#pragma comment(lib, "user32.lib")
#pragma comment(lib, "d3d12.lib")
#pragma comment(lib, "dxgi.lib")

using Microsoft::WRL::ComPtr;   // RAII for COM objects: releases automatically.

namespace {

    constexpr UINT   kBackBufferCount = 2;
    constexpr UINT   kWidth = 1280;
    constexpr UINT   kHeight = 720;

    struct Graphics
    {
        ComPtr<ID3D12Device>              device;
        ComPtr<ID3D12CommandQueue>        queue;
        ComPtr<ID3D12CommandAllocator>    allocator;
        ComPtr<ID3D12GraphicsCommandList> cmdList;
        ComPtr<IDXGISwapChain3>           swapChain;
        ComPtr<ID3D12DescriptorHeap>      rtvHeap;
        ComPtr<ID3D12Fence>               fence;
        HANDLE fenceEvent = nullptr;
        UINT64 fenceValue = 0;
        UINT   rtvIncrement = 0;
        UINT   frameIndex = 0;
    } g;

    double NowSeconds()
    {
        using namespace std::chrono;
        return duration<double>(steady_clock::now().time_since_epoch()).count();
    }

    LRESULT CALLBACK WindowProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
    {
        switch (msg)
        {
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
        default:
            return DefWindowProcW(hwnd, msg, wParam, lParam);
        }
    }

    bool InitGraphics(HWND hwnd)
    {
        // --- Optional: debug layer. Gives validation errors and GPU-side warnings. ---
        bool debugOn = false;
#if defined(_DEBUG)
        ComPtr<ID3D12Debug> dbg;
        if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&dbg))))
        {
            dbg->EnableDebugLayer();
            debugOn = true;
            std::printf("[gfx] D3D12 debug layer ON.\n");
        }
        else
        {
            std::printf("[gfx] Debug layer unavailable (Windows 'Graphics Tools' optional feature).\n");
        }
#endif

        ComPtr<IDXGIFactory4> factory;
        const HRESULT hrFactory = debugOn
            ? CreateDXGIFactory2(DXGI_CREATE_FACTORY_DEBUG, IID_PPV_ARGS(&factory))
            : CreateDXGIFactory1(IID_PPV_ARGS(&factory));
        if (FAILED(hrFactory))
            return false;

        // --- Pick the first real (non-software) GPU and print its name. ---
        ComPtr<IDXGIAdapter1> adapter;
        for (UINT i = 0; factory->EnumAdapters1(i, &adapter) != DXGI_ERROR_NOT_FOUND; ++i)
        {
            DXGI_ADAPTER_DESC1 desc{};
            adapter->GetDesc1(&desc);
            if (desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) { adapter.Reset(); continue; }
            std::printf("[gfx] Adapter: %ls\n", desc.Description);
            break;
        }

        if (FAILED(D3D12CreateDevice(adapter.Get(), D3D_FEATURE_LEVEL_11_0,
            IID_PPV_ARGS(&g.device))))
            return false;
        std::printf("[gfx] Device created.\n");

        // --- Command queue: the conveyor belt. ---
        D3D12_COMMAND_QUEUE_DESC qd{};
        qd.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
        if (FAILED(g.device->CreateCommandQueue(&qd, IID_PPV_ARGS(&g.queue))))
            return false;

        if (FAILED(g.device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT,
            IID_PPV_ARGS(&g.allocator))))
            return false;
        if (FAILED(g.device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT,
            g.allocator.Get(), nullptr,
            IID_PPV_ARGS(&g.cmdList))))
            return false;
        g.cmdList->Close();   // Lists are born open; the frame loop expects closed.

        // --- Swap chain bound to our window. ---
        DXGI_SWAP_CHAIN_DESC1 sd{};
        sd.BufferCount = kBackBufferCount;
        sd.Width = kWidth;
        sd.Height = kHeight;
        sd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        sd.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
        sd.SampleDesc.Count = 1;

        ComPtr<IDXGISwapChain1> sc1;
        if (FAILED(factory->CreateSwapChainForHwnd(g.queue.Get(), hwnd, &sd,
            nullptr, nullptr, &sc1)))
            return false;
        if (FAILED(sc1.As(&g.swapChain)))
            return false;
        g.frameIndex = g.swapChain->GetCurrentBackBufferIndex();

        // --- RTV heap: one typed pointer per back buffer. ---
        D3D12_DESCRIPTOR_HEAP_DESC hd{};
        hd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
        hd.NumDescriptors = kBackBufferCount;
        if (FAILED(g.device->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&g.rtvHeap))))
            return false;
        g.rtvIncrement = g.device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

        for (UINT i = 0; i < kBackBufferCount; ++i)
        {
            ComPtr<ID3D12Resource> backBuffer;
            if (FAILED(g.swapChain->GetBuffer(i, IID_PPV_ARGS(&backBuffer))))
                return false;
            D3D12_CPU_DESCRIPTOR_HANDLE rtv = g.rtvHeap->GetCPUDescriptorHandleForHeapStart();
            rtv.ptr += i * g.rtvIncrement;
            g.device->CreateRenderTargetView(backBuffer.Get(), nullptr, rtv);
        }

        // --- Fence: the CPU/GPU handshake. ---
        if (FAILED(g.device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&g.fence))))
            return false;
        g.fenceEvent = CreateEventW(nullptr, FALSE, FALSE, nullptr);

        std::printf("[gfx] Swap chain ready: %u back buffers, %ux%u.\n",
            kBackBufferCount, kWidth, kHeight);
        return true;
    }

    void RenderFrame(double timeSec)
    {
        // --- Record this frame's commands. ---
        g.allocator->Reset();
        g.cmdList->Reset(g.allocator.Get(), nullptr);

        ComPtr<ID3D12Resource> backBuffer;
        g.swapChain->GetBuffer(g.frameIndex, IID_PPV_ARGS(&backBuffer));

        // Back buffers live in PRESENT state between frames; transition to drawable.
        D3D12_RESOURCE_BARRIER barrier{};
        barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        barrier.Transition.pResource = backBuffer.Get();
        barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_PRESENT;
        barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;
        g.cmdList->ResourceBarrier(1, &barrier);

        D3D12_CPU_DESCRIPTOR_HANDLE rtv = g.rtvHeap->GetCPUDescriptorHandleForHeapStart();
        rtv.ptr += g.frameIndex * g.rtvIncrement;

        // A slowly breathing ocean blue. If this animates, your loop is alive.
        const float t = (float)timeSec;
        const float clearColor[4] = {
            0.00f,
            0.10f + 0.05f * sinf(t * 0.9f),
            0.25f + 0.08f * sinf(t * 0.6f),
            1.00f
        };
        g.cmdList->ClearRenderTargetView(rtv, clearColor, 0, nullptr);

        // Hand the buffer back to the presentation engine.
        barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
        barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_PRESENT;
        g.cmdList->ResourceBarrier(1, &barrier);
        g.cmdList->Close();

        // --- Submit and present. ---
        ID3D12CommandList* lists[] = { g.cmdList.Get() };
        g.queue->ExecuteCommandLists(1, lists);
        g.swapChain->Present(1, 0);   // interval 1 = vsync

        // --- Wait for the GPU to finish before we reuse this frame's resources. ---
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
        CloseHandle(g.fenceEvent);
        std::printf("[gfx] GPU idle, shutdown clean.\n");
    }

} // namespace

int main()
{
    std::printf("CFD_but_actually_fun: opening window...\n");

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

        RenderFrame(NowSeconds() - start);
    }

    ShutdownGraphics();
    std::printf("Window closed cleanly. Goodbye.\n");
    return 0;
}
