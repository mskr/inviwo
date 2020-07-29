/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2014-2020 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

// Owned by the TubeRendering Processor

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
in vec3 startPos_;
in vec3 endPos_;
in vec3 gEndplanes[2];
in float radius0_;
in float radius1_;

float dot2(vec3 v) { return dot(v, v); }

vec4 coneIntersect( in vec3  ro, in vec3  rd, in vec3  pa, in vec3  pb, in float ra, in float rb )
{
    vec3  ba = pb - pa;
    vec3  oa = ro - pa;
    vec3  ob = ro - pb;
    float m0 = dot(ba,ba);
    float m1 = dot(oa,ba);
    float m2 = dot(rd,ba);
    float m3 = dot(rd,oa);
    float m5 = dot(oa,oa);
    float m9 = dot(ob,ba); 
    
    // caps
    if( m1<0.0 )
    {
        if( dot2(oa*m2-rd*m1)<(ra*ra*m2*m2) ) // delayed division
            return vec4(-m1/m2,-ba*inversesqrt(m0));
    }
    else if( m9>0.0 )
    {
        float t = -m9/m2;                     // NOT delayed division
        if( dot2(ob+rd*t)<(rb*rb) )
            return vec4(t,ba*inversesqrt(m0));
    }
    
    // body
    float rr = ra - rb;
    float hy = m0 + rr*rr;
    float k2 = m0*m0    - m2*m2*hy;
    float k1 = m0*m0*m3 - m1*m2*hy + m0*ra*(rr*m2*1.0        );
    float k0 = m0*m0*m5 - m1*m1*hy + m0*ra*(rr*m1*2.0 - m0*ra);
    float h = k1*k1 - k2*k0;
    if( h<0.0 ) return vec4(-1.0); //no intersection
    float t = (-k1-sqrt(h))/k2;
    float y = m1 + t*m2;
    if( y<0.0 || y>m0 ) return vec4(-1.0); //no intersection
    return vec4(t, normalize(m0*(m0*(oa+t*rd)+rr*ba*ra)-ba*hy*y));
}


void main() {

    if (startPos_ == endPos_) discard;

    vec3 tubeDir = normalize(endPos_ - startPos_);
    vec3 start = startPos_ - .1 * tubeDir;
    vec3 end = endPos_ + .1 * tubeDir;
    float r0 = radius0_ - .1 * radius0_;
    float r1 = radius1_ + .1 * radius1_;

    vec3 camPos = (camera.viewToWorld * vec4(0,0,0,1)).xyz;
    vec3 dir = normalize(worldPos_ - camPos);

    vec4 tnor = coneIntersect(worldPos_, dir, start, end, r0, r1);

    float d = tnor.x;
    if (d < 0.0){
        discard;
        return;
    }

    vec3 hitPoint = worldPos_ + d * dir;
    if (dot(hitPoint - startPos_, gEndplanes[0]) < 0.0) {
        discard;
        return;
    }
    if (dot(hitPoint - endPos_, gEndplanes[1]) > 0.0) {
        discard;
        return;
    }

    vec3 light = APPLY_LIGHTING(lighting, color_.rgb, color_.rgb, vec3(1.0f), hitPoint, tnor.yzw, dir);
    vec4 color = vec4(light, color_.a);
    vec4 ndc = camera.worldToClip * vec4(hitPoint, 1);
    float depth = ((ndc.z / ndc.w) + 1 ) / 2;

#if defined(USE_FRAGMENT_LIST)
    // fragment list rendering
    if (color.a > 0.0) {
        ivec2 coords = ivec2(gl_FragCoord.xy);

        abufferRender(coords, depth, color);
    }
    discard;

#else  // USE_FRAGMENT_LIST

    gl_FragDepth = depth;
    FragData0 = vec4(color.rgb, color_.a);
    PickingData = pickColor_;
    
#endif // not USE_FRAGMENT_LIST
}