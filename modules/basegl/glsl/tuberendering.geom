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

layout(triangle_strip, max_vertices = 24) out;

uniform GeometryParameters geometry;
uniform CameraParameters camera;

in vec4 vColor_[SIZE];
flat in float vRadius_[SIZE];
flat in uint pickID_[SIZE];

flat out vec4 color0_;
flat out vec4 color1_;
flat out vec4 pickColor_;
out vec3 worldPos_;
flat out vec3 startPos_;
flat out vec3 endPos_;
flat out vec3 gEndplanes[2];
flat out float radius0_;
flat out float radius1_;

vec3 prismoid[8];
vec4 color[2];
vec4 pickColor;
vec3 startPos;
vec3 endPos;
vec3 capNormals[2];
float radius[2];

void emitVertex(int a) { 
    gl_Position = camera.worldToClip * vec4(prismoid[a], 1.0);  
    color0_ = color[0];
    color1_ = color[1];
    pickColor_ = pickColor;
    worldPos_ = prismoid[a];
    startPos_ = startPos;
    endPos_ = endPos;
    gEndplanes[0] = capNormals[0];
    gEndplanes[1] = capNormals[1];
    radius0_ = radius[0];
    radius1_ = radius[1];
    EmitVertex();
}

void emitFace(int a, int b, int c, int d) {
    emitVertex(a);
    emitVertex(b);
    emitVertex(c);
    emitVertex(d);
    EndPrimitive(); 
}

// v should be normalized
vec3 findOrthogonalVector(vec3 v) {
    vec3 A = normalize((camera.viewToWorld * vec4(1,0,0,0)).xyz);
    if (abs(dot(v,A)) > 0.5) {
        return cross(v,A);
    } else {
        vec3 B = normalize((camera.viewToWorld * vec4(0,0,1,0)).xyz);
        return cross(v,B);
    }
}

// 2D cross-section space
vec2 project(vec3 p, vec3 radialDir, vec3 origin) {
    vec3 tubeDir = normalize(endPos-startPos);
    vec3 tmp = (p - origin);
    return vec2(dot(tmp, tubeDir), dot(tmp, radialDir));
}

vec3 unproject(vec2 p, vec3 radialDir, vec3 origin) {
    vec3 tubeDir = normalize(endPos-startPos);    
    return origin + tubeDir * p.x + radialDir * p.y;
}

vec2 intersect(vec2 A, vec2 B, vec2 C, vec2 D) {
    float x1 = A.x, y1 = A.y, x2 = B.x, y2 = B.y;
    float x3 = C.x, y3 = C.y, x4 = D.x, y4 = D.y;
    return vec2(
        ((x1*y2-y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4)),
        ((x1*y2-y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4)))
        / ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));
}

// Search intersections of two cones in 2D cross section space
void connect(
    vec3 nextPos, // far end of adjacent cone
    vec3 radialDir, // defines cross section
    vec3 nextRadialDir, // must lie in defined cross section
    float radius[2], // cone radii
    float nextRadius, // adjacent cone radius at far end
    vec3 projOrigin, // previous cone => startPos, next cone => endPos
    out vec2 E, // intersection of walls on one side
    out vec2 F, // intersection of walls on other side to define interface line
    out vec2 G, 
    out vec2 H)
{
    vec2 A = project(startPos +radialDir*radius[0],      radialDir, projOrigin);
    vec2 B = project(endPos   +radialDir*radius[1],      radialDir, projOrigin);
    vec2 C = project(endPos   +nextRadialDir*radius[1],  radialDir, projOrigin);
    vec2 D = project(nextPos  +nextRadialDir*nextRadius, radialDir, projOrigin);
    vec2 A_ = project(startPos-radialDir*radius[0],      radialDir, projOrigin);
    vec2 B_ = project(endPos  -radialDir*radius[1],      radialDir, projOrigin);
    vec2 C_ = project(endPos  -nextRadialDir*radius[1],  radialDir, projOrigin);
    vec2 D_ = project(nextPos -nextRadialDir*nextRadius, radialDir, projOrigin);
    E = intersect(A,B,C,D);
    F = intersect(A_,B_,C_,D_);
    G = intersect(E,F,vec2(-1,0),vec2(1,0));
    H = intersect(E,F,project(projOrigin,radialDir,projOrigin), project(nextPos,radialDir,projOrigin));
}


/*  Corners of the prismoid;
 *   
 *        7------ 4                     
 *       /|  p2  /|                 
 *      / |  *  / |                 
 *     /  6----/--5                 
 *    3-------0  /                  
 *    | / p1  | /                   
 *    |/  *   |/                    
 *    2-------1
 *
 * if we let p2-p1 -> z i -> x, and k -> y
 */

void main() {
    color[0] = vColor_[BEGIN];
    color[1] = vColor_[END];
    radius[0] = vRadius_[BEGIN];
    radius[1] = vRadius_[END];
    pickColor = vec4(pickingIndexToColor(pickID_[BEGIN]), pickID_[BEGIN] == 0 ? 0.0 : 1.0);
    startPos = gl_in[BEGIN].gl_Position.xyz;
    endPos = gl_in[END].gl_Position.xyz;

#ifdef HAS_ADJACENCY
    vec3 prevPos = gl_in[0].gl_Position.xyz;
    vec3 nextPos = gl_in[3].gl_Position.xyz;
    float prevRadius = vRadius_[0];
    float nextRadius = vRadius_[3];
#else
    vec3 prevPos = startPos;
    vec3 nextPos = endPos;
    float prevRadius = radius[0];
    float nextRadius = radius[1];
#endif

    if (startPos == endPos) return; // zero size segment

    vec3 tubeDir = normalize(endPos-startPos);
    vec3 radialDir = findOrthogonalVector(tubeDir);
  
    vec3 prevDir = startPos-prevPos;
    vec3 nextDir = nextPos-endPos;
    capNormals[0] = normalize(tubeDir + (prevDir != vec3(0) ? normalize(prevDir) : prevDir));
    capNormals[1] = normalize(tubeDir + (nextDir != vec3(0) ? normalize(nextDir) : nextDir));

    // vec3 radialDirCross = cross(tubeDir, radialDir);

    // vec3 nextTubeDir = normalize(nextPos-endPos);
    // vec3 nextRadialDir = normalize(cross(cross(nextTubeDir, radialDir), nextTubeDir));
    // vec3 nextRadialDirCross = cross(nextTubeDir, nextRadialDir);

    // vec3 prevTubeDir = normalize(prevPos-startPos);
    // vec3 prevRadialDir = normalize(cross(cross(prevTubeDir, radialDir), prevTubeDir));
    // vec3 prevRadialDirCross = cross(prevTubeDir, prevRadialDir);

    // vec2 E, F, G, H;
    // connect(nextPos, radialDir, nextRadialDir, radius, nextRadius, endPos, E, F, G, H);
    // vec3 correctedEnd = unproject(G, radialDir, endPos);
    // vec3 planeVec1 = unproject(F, radialDir, endPos) - unproject(E, radialDir, endPos);
    // connect(prevPos, radialDir, prevRadialDir, radius, prevRadius, startPos, E, F, G, H);
    // vec3 correctedStart = unproject(H, radialDir, endPos);
    // connect(nextPos, radialDirCross, nextRadialDirCross, radius, nextRadius, endPos, E, F, G, H);
    // vec3 planeVec2 = unproject(F, radialDirCross, endPos) - unproject(E, radialDirCross, endPos);
    // vec3 connectionNormal = cross(normalize(planeVec1), normalize(planeVec2));

    // startPos = correctedStart;
    // endPos = correctedEnd;


    float scale = 2;

    // Compute cap face 1 of 2:
    vec3 k = scale * radius[0] * normalize(cross(radialDir, capNormals[0])); 
    vec3 i = scale * radius[0] * normalize(cross(k, capNormals[0])); 
    prismoid[0] = startPos + i + k;
    prismoid[1] = startPos + i - k;
    prismoid[2] = startPos - i - k;
    prismoid[3] = startPos - i + k;

    // Compute cap face 2 of 2:
    k = scale * radius[1] * normalize(cross(radialDir, capNormals[1])); 
    i = scale * radius[1] * normalize(cross(k, capNormals[1])); 
    prismoid[4] = endPos + i + k;
    prismoid[5] = endPos + i - k;
    prismoid[6] = endPos - i - k;
    prismoid[7] = endPos - i + k;

    // Emit the six faces of the prismoid:
    emitFace(0,1,3,2); 
    emitFace(5,4,6,7);
    emitFace(4,5,0,1); 
    emitFace(3,2,7,6);
    emitFace(0,3,4,7); 
    emitFace(2,1,6,5);
}
