cbuffer RootConstants : register(b0)
{
    row_major float4x4 mvpWater;
    row_major float4x4 mvpModel;
    row_major float4x4 model;
    float4 lightDir;
    float4 camPosTime;
};

struct VSOut
{
    float4 clip : SV_Position;
    float3 world : TEXCOORD0;
    float3 normal : TEXCOORD1;
};

float4 main(VSOut input) : SV_Target
{
    float3 N = normalize(input.normal);
    float3 L = normalize(lightDir.xyz);
    float3 V = normalize(camPosTime.xyz - input.world);
    float3 H = normalize(L + V);

    float ndl = saturate(dot(N, L));

    float3 deep = float3(0.01, 0.10, 0.20);
    float3 shallow = float3(0.04, 0.36, 0.44);
    float3 base = lerp(deep, shallow, ndl);

    float spec = pow(saturate(dot(N, H)), 120.0) * 1.6;
    float foam = smoothstep(0.30, 0.48, input.world.y);

    float3 col = base * (0.35 + 0.65 * ndl) + spec;
    col = lerp(col, float3(0.85, 0.92, 0.97), foam * 0.65);
    return float4(col, 1.0);
}

float4 psCrate(VSOut input) : SV_Target
{
    float3 N = normalize(input.normal);
    float3 L = normalize(lightDir.xyz);
    float ndl = saturate(dot(N, L));

    float3 albedo = float3(0.45, 0.30, 0.16);
    float3 col = albedo * (0.25 + 0.75 * ndl);

    float3 V = normalize(camPosTime.xyz - input.world);
    float3 H = normalize(L + V);
    col += pow(saturate(dot(N, H)), 40.0) * 0.15;

    // Wet band: darker wood below the waterline. Cheap, reads instantly.
    float wet = smoothstep(0.15, -0.05, input.world.y);
    col = lerp(col, col * 0.35, wet);
    return float4(col, 1.0);
}
