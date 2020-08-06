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

#ifdef HAS_ADJACENCY
#define SIZE 4
#define BEGIN 1
#define END 2
layout(lines_adjacency) in;
#else
#define SIZE 2
#define BEGIN 0
#define END 1
layout(lines) in;
#endif

#define NGON 16

layout(triangle_strip, max_vertices = NGON*2+2) out;

uniform GeometryParameters geometry;
uniform CameraParameters camera;

in vec4 vColor_[SIZE];
flat in float vRadius_[SIZE];
flat in uint pickID_[SIZE];

out vec4 color_;
flat out vec4 pickColor_;
out vec3 worldPos_;
out vec3 worldNormal_;

vec3 ngonTube[NGON*2];

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

vec3 vertexNormal(int vert, int opp, int prev, int next) {
    return cross(
            normalize(ngonTube[opp] - ngonTube[vert]),
            normalize(ngonTube[next] - ngonTube[prev]));
}

void main() {
    color[0] = vColor_[BEGIN];
    color[1] = vColor_[END];
    float radius[2];
    radius[0] = vRadius_[BEGIN];
    radius[1] = vRadius_[END];
    pickColor = vec4(pickingIndexToColor(pickID_[BEGIN]), pickID_[BEGIN] == 0 ? 0.0 : 1.0);
    vec3 startPos = gl_in[BEGIN].gl_Position.xyz;
    vec3 endPos = gl_in[END].gl_Position.xyz;
#ifdef HAS_ADJACENCY
    vec3 prevPos = gl_in[0].gl_Position.xyz;
    vec3 nextPos = gl_in[3].gl_Position.xyz;
#else
    vec3 prevPos = startPos;
    vec3 nextPos = endPos;
#endif

    if (startPos == endPos) return; // zero size segment

    vec3 tubeDir = normalize(endPos-startPos);
    vec3 radialDir = findOrthogonalVector(tubeDir);
  
    vec3 prevDir = startPos-prevPos;
    vec3 nextDir = nextPos-endPos;
    vec3 capNormals[2];
    capNormals[0] = normalize(tubeDir + (prevDir != vec3(0) ? normalize(prevDir) : prevDir));
    capNormals[1] = normalize(tubeDir + (nextDir != vec3(0) ? normalize(nextDir) : nextDir));

    vec3 x = findOrthogonalVector(capNormals[0]);
    vec3 y = cross(x, capNormals[0]);
    float twopi = 6.28318530718;
    float step = twopi / NGON;

    for(int i = 0; i < NGON; i++) {
        ngonTube[i] = startPos + radius[0] * sin(i*step) * x + radius[0] * cos(i*step) * y;
    }

    x = findOrthogonalVector(capNormals[1]);
    y = cross(x, capNormals[1]);

    for(int i = NGON; i < 2*NGON; i++) {
        ngonTube[i] = endPos + radius[1] * sin(i*step) * x + radius[1] * cos(i*step) * y;
    }

    // sides
    worldNormal_ = vertexNormal(NGON, 0, 1, NGON-1);
    emitVertex(0);
    for(int i = 0; i < NGON; i++) {
        //emitFace(i, NGON+i, (i+1)%NGON, NGON+((i+1)%NGON));

        int vert = NGON+i;
        int opp = i;
        int prev = NGON + ((NGON+i-1) % NGON);
        int next = NGON + ((NGON+i+1) % NGON);
        worldNormal_ = vertexNormal(opp, vert, prev, next);
        emitVertex(vert);

        vert = (i+1)%NGON;
        opp = next;
        prev = i;
        next = (i+2)%NGON;
        worldNormal_ = vertexNormal(vert, opp, prev, next);
        emitVertex(vert);
    }
    worldNormal_ = vertexNormal(NGON, 0, NGON+1, NGON*2-1);
    emitVertex(NGON);
    EndPrimitive();

    // front
    for (int i = 0; i < NGON/2; i++) {
        emitVertex((NGON-i)%NGON);
        emitVertex(i+1);
    }
    EndPrimitive();

    // back
    for (int i = 0; i < NGON/2; i++) {
        emitVertex(i==0? NGON: 2*NGON-i);
        emitVertex(NGON+1+i);
    }
    EndPrimitive();
}
