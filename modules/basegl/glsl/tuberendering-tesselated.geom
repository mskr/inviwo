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
#include "utils/pickingutils.glsl"

#define SIZE 4
#define BEGIN 1
#define END 2
layout(lines_adjacency) in;

#define NGON 16
    
float TWOPI = 6.28318530718;

layout(triangle_strip, max_vertices = NGON*2+2) out;

uniform GeometryParameters geometry;
uniform CameraParameters camera;

in vec4 vColor_[SIZE];
flat in float vRadius_[SIZE];
flat in vec3 vNormal_[SIZE];
flat in vec3 vTexCoord_[SIZE];
flat in uint pickID_[SIZE];

out vec4 color_;
flat out vec4 pickColor_;
out vec3 worldPos_;
out vec3 worldNormal_;

vec3 ngonTube[NGON*2];

vec3 nborNgons[NGON*2];

vec4 color[2];
vec4 pickColor;

void emitVertex(int a) { 
    gl_Position = camera.worldToClip * vec4(ngonTube[a], 1.0);  
    color_ = color[a < NGON ? 0 : 1];
    pickColor_ = pickColor;
    worldPos_ = ngonTube[a];
    EmitVertex();
}

void emitFace(int a, int b, int c, int d) {
    worldNormal_ = normalize(cross(
        normalize(ngonTube[a] - ngonTube[b]), normalize(ngonTube[c] - ngonTube[b])));
    emitVertex(a);
    emitVertex(b);
    emitVertex(c);
    emitVertex(d);
    EndPrimitive(); 
}

// v should be normalized
vec3 findOrthogonalVector(vec3 v) {
    vec3 b = cross(v, vec3(0, 0, 1));
    if (dot(b, b) < 0.01) {
        b = cross(v, vec3(0, 1, 0));
    }
    return b;
}

void main() {
    color[0] = vColor_[BEGIN];
    color[1] = vColor_[END];
    float radius[2];
    radius[0] = vRadius_[BEGIN];
    radius[1] = vRadius_[END];
    if (radius[0] == 0) { radius[0] = .0001; color[0].a = 0; }
    if (radius[1] == 0) { radius[1] = .0001; color[1].a = 0; }
    pickColor = vec4(pickingIndexToColor(pickID_[BEGIN]), pickID_[BEGIN] == 0 ? 0.0 : 1.0);
    vec3 startPos = gl_in[BEGIN].gl_Position.xyz;
    vec3 endPos = gl_in[END].gl_Position.xyz;
    vec3 startNormal = vNormal_[1];
    vec3 endNormal = vNormal_[2];

    vec3 prevPos = gl_in[0].gl_Position.xyz;
    vec3 nextPos = gl_in[3].gl_Position.xyz;
    float prevRadius = vRadius_[0];
    float nextRadius = vRadius_[3];
    vec3 prevNormal = vNormal_[0];
    vec3 nextNormal = vNormal_[3];

    bool firstSegment = startPos == prevPos;
    bool lastSegment = endPos == nextPos;

    if (firstSegment) prevPos = startPos - (endPos - startPos);
    if (lastSegment) nextPos = endPos + (endPos - startPos);

    vec3 startHalftubeNormal = vTexCoord_[1];
    vec3 endHalftubeNormal = vTexCoord_[2];

    if (startHalftubeNormal != vec3(0)) {
        // for halftubes start here
    }

    vec3 x = findOrthogonalVector(startNormal);
    vec3 y = cross(x, startNormal);

    vec3 xp = findOrthogonalVector(prevNormal);
    vec3 yp = cross(xp, prevNormal);

    float step = TWOPI / NGON;

    for(int i = 0; i < NGON; i++) {
        ngonTube[i] = startPos + radius[0] * sin(i*step) * x + radius[0] * cos(i*step) * y;
        nborNgons[i] = prevPos + prevRadius * sin(i*step) * xp + prevRadius * cos(i*step) * yp;
    }

    x = findOrthogonalVector(endNormal);
    y = cross(x, endNormal);

    vec3 xn = findOrthogonalVector(nextNormal);
    vec3 yn = cross(xn, nextNormal);

    for(int i = NGON; i < 2*NGON; i++) {
        ngonTube[i] = endPos + radius[1] * sin(i*step) * x + radius[1] * cos(i*step) * y;
        nborNgons[i] = nextPos + nextRadius * sin(i*step) * xn + nextRadius * cos(i*step) * yn;
    }

    if (startPos != endPos) {

        // sides
        worldNormal_ = cross(normalize(ngonTube[NGON-1] - ngonTube[1]),
            normalize( normalize(ngonTube[0] - nborNgons[0]) + normalize(ngonTube[NGON] - ngonTube[0]) ));
        emitVertex(0);
        for(int i = 0; i < NGON; i++) {
            //emitFace(i, NGON+i, (i+1)%NGON, NGON+((i+1)%NGON));

            int vert = NGON+i;
            int opp = i;
            int prev = NGON + ((NGON+i-1) % NGON);
            int next = NGON + ((NGON+i+1) % NGON);
            worldNormal_ = cross(normalize(ngonTube[prev] - ngonTube[next]),
                normalize( normalize(nborNgons[vert] - ngonTube[vert]) + normalize(ngonTube[vert] - ngonTube[opp]) ));
            emitVertex(vert);

            vert = (i+1)%NGON;
            opp = next;
            prev = i;
            next = (i+2)%NGON;
            worldNormal_ = cross(normalize(ngonTube[prev] - ngonTube[next]),
                normalize( normalize(ngonTube[vert] - nborNgons[vert]) + normalize(ngonTube[opp] - ngonTube[vert]) ));
            emitVertex(vert);
        }
        worldNormal_ = cross(normalize(ngonTube[NGON-1] - ngonTube[1]),
            normalize( normalize(nborNgons[NGON] - ngonTube[NGON]) + normalize(ngonTube[NGON] - ngonTube[0]) ));
        emitVertex(NGON);
        EndPrimitive();

        if (firstSegment) {
            // front
            // for (int i = 0; i < NGON/2; i++) {
            //     emitVertex((NGON-i)%NGON);
            //     emitVertex(i+1);
            // }
            // EndPrimitive();
        }
        
    } else {

        // // back
        // for (int i = 0; i < NGON/2; i++) {
        //     emitVertex(i==0? NGON: 2*NGON-i);
        //     emitVertex(NGON+1+i);
        // }
        // EndPrimitive();

    }
}
