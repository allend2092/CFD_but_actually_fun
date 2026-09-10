cbuffer RootConstants : register(b0)
{
    row_major float4x4 mvp;
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

    // Slopes facing the light read as shallow, lit water; flat faces read deep.
    float3 deep = float3(0.01, 0.10, 0.20);
    float3 shallow = float3(0.04, 0.36, 0.44);
    float3 base = lerp(deep, shallow, ndl);

    float spec = pow(saturate(dot(N, H)), 120.0) * 1.6;   // Sun glint.
    float foam = smoothstep(0.30, 0.48, input.world.y);   // Crest foam.

    float3 col = base * (0.35 + 0.65 * ndl) + spec;
    col = lerp(col, float3(0.85, 0.92, 0.97), foam * 0.65);
    return float4(col, 1.0);
}