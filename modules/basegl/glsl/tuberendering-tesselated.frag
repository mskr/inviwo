#include "utils/structs.glsl"
#include "utils/shading.glsl"

#ifdef USE_FRAGMENT_LIST
#include "oit/abufferlinkedlist.glsl"

// this is important for the occlusion query
layout(early_fragment_tests) in;

layout(pixel_center_integer) in vec4 gl_FragCoord;
#endif

uniform LightParameters lighting;
uniform CameraParameters camera;

in vec4 color_;
flat in vec4 pickColor_;
in vec3 worldPos_;
in vec3 worldNormal_;

vec4 getColor(vec3 pos, vec3 normal, vec3 viewDir) {
    vec3 rgb = APPLY_LIGHTING(lighting, color_.rgb, color_.rgb, vec3(1.0f), pos, normal, viewDir);
    return vec4(rgb, color_.a);
}

void main() {

    vec3 camPos = (camera.viewToWorld * vec4(0,0,0,1)).xyz;
    vec3 dir = normalize(worldPos_ - camPos);
    vec4 color = getColor(worldPos_, worldNormal_, -dir);

#if defined(USE_FRAGMENT_LIST)
    if (color.a > 0.0) {
        ivec2 coords = ivec2(gl_FragCoord.xy);
        float depth = gl_FragCoord.z;
        abufferRender(coords, depth, color);
    }
    discard;

#else  // USE_FRAGMENT_LIST

    FragData0 = color;
    PickingData = pickColor_;
    
#endif // not USE_FRAGMENT_LIST
}