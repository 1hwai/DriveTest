#version 330 core

in vec3 vWorldPosition;
in vec3 vWorldNormal;

uniform vec3 uLightDirection;
uniform vec3 uCameraPosition;

out vec4 FragColor;

void main() {
    const vec3 baseColor = vec3(1.0);

    vec3 normal = normalize(vWorldNormal);
    vec3 lightDirection = normalize(-uLightDirection);

    float diffuse = max(dot(normal, lightDirection), 0.0);

    vec3 viewDirection =
        normalize(uCameraPosition - vWorldPosition);

    vec3 halfVector =
        normalize(lightDirection + viewDirection);

    float specular =
        pow(max(dot(normal, halfVector), 0.0), 32.0);

    vec3 color =
        baseColor * (0.18 + diffuse * 0.72) +
        vec3(0.18) * specular;

    FragColor = vec4(color, 1.0);
}