#version 330 core

in vec3 vWorldPosition;
in vec3 vWorldNormal;
in vec2 vTexCoord;

uniform vec3 uLightDirection;
uniform vec3 uCameraPosition;
uniform vec3 uBaseColor;
uniform sampler2D uBaseColorTexture;
uniform sampler2D uSecondaryTexture;
uniform int uUseTexture;
uniform int uSurfaceType;
uniform float uSurfaceBlend;

out vec4 FragColor;

float hash21(vec2 p) {
    p = fract(p * vec2(123.34, 456.21));
    p += dot(p, p + 45.32);
    return fract(p.x * p.y);
}

float noise2(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float a = hash21(i);
    float b = hash21(i + vec2(1.0, 0.0));
    float c = hash21(i + vec2(0.0, 1.0));
    float d = hash21(i + vec2(1.0, 1.0));
    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

void main() {
    vec3 normal = normalize(vWorldNormal);
    vec3 lightDirection = normalize(-uLightDirection);

    float diffuse = max(dot(normal, lightDirection), 0.0);

    vec3 viewDirection =
        normalize(uCameraPosition - vWorldPosition);

    vec3 halfVector =
        normalize(lightDirection + viewDirection);

    float specular =
        pow(max(dot(normal, halfVector), 0.0), 32.0);

    vec3 albedo = uBaseColor;
    vec2 worldXZ = vWorldPosition.xz;

    if (uSurfaceType == 1) {
        vec3 dirt = texture(uBaseColorTexture, vTexCoord).rgb;
        float fineNoise = noise2(worldXZ * 0.22);
        vec3 grass = mix(vec3(0.13, 0.21, 0.09), vec3(0.22, 0.29, 0.13), fineNoise);
        albedo = mix(grass, dirt, 0.72);
        albedo = mix(albedo, vec3(0.28, 0.26, 0.22), smoothstep(0.68, 0.88, 1.0 - normal.y));
    } else if (uSurfaceType == 2) {
        albedo = texture(uBaseColorTexture, vTexCoord).rgb;
    } else if (uSurfaceType == 3) {
        albedo = texture(uBaseColorTexture, vTexCoord).rgb;
    } else if (uSurfaceType == 4) {
        vec3 asphalt = texture(uBaseColorTexture, vTexCoord).rgb;
        vec3 dirt = texture(uSecondaryTexture, vTexCoord).rgb;
        albedo = mix(asphalt, dirt, clamp(uSurfaceBlend, 0.0, 1.0));
    } else if (uSurfaceType == 5) {
        float rockNoise = noise2(worldXZ * 0.42) * 0.65 + noise2(worldXZ * 1.7) * 0.35;
        albedo = mix(vec3(0.22, 0.20, 0.17), vec3(0.43, 0.39, 0.32), rockNoise);
    } else if (uUseTexture != 0) {
        vec4 texel = texture(uBaseColorTexture, vTexCoord);
        if (texel.a < 0.38) discard;
        albedo *= texel.rgb;
    }

    vec3 color = albedo * (0.32 + diffuse * 0.78);
    if (uSurfaceType == 0)
        color += vec3(0.22) * specular;

    FragColor = vec4(min(color, vec3(1.0)), 1.0);
}
