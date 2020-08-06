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

flat in vec4 color0_;
flat in vec4 color1_;
flat in vec4 pickColor_;
in vec3 worldPos_;
in vec3 startPos_;
in vec3 endPos_;
in vec3 gEndplanes[2];
flat in float radius0_;
flat in float radius1_;

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

vec4 iRoundedCone( in vec3 ro, in vec3 rd, in vec3 pa, in vec3 pb, in float ra, in float rb )
{
    vec3  ba = pb - pa;
    vec3  oa = ro - pa;
    vec3  ob = ro - pb;
    float rr = ra - rb;
    float m0 = dot(ba,ba);
    float m1 = dot(ba,oa);
    float m2 = dot(ba,rd);
    float m3 = dot(rd,oa);
    float m5 = dot(oa,oa);
    float m6 = dot(ob,rd);
    float m7 = dot(ob,ob);
    
    // body
    float d2 = m0-rr*rr;
    float k2 = d2    - m2*m2;
    float k1 = d2*m3 - m1*m2 + m2*rr*ra;
    float k0 = d2*m5 - m1*m1 + m1*rr*ra*2.0 - m0*ra*ra;
    float h = k1*k1 - k0*k2;
    if( h<0.0) return vec4(-1.0);
    float t = (-sqrt(h)-k1)/k2;
  //if( t<0.0 ) return vec4(-1.0);
    float y = m1 - ra*rr + t*m2;
    if( y>0.0 && y<d2 ) return vec4(t, normalize(d2*(oa+t*rd)-ba*y));

    // caps
    float h1 = m3*m3 - m5 + ra*ra;
    float h2 = m6*m6 - m7 + rb*rb;
    if( max(h1,h2)<0.0 ) return vec4(-1.0);
    vec4 r = vec4(1e20);
    if( h1>0.0 )
    {        
        t = -m3 - sqrt( h1 );
        r = vec4( t, (oa+t*rd)/ra );
    }
    if( h2>0.0 )
    {
        t = -m6 - sqrt( h2 );
        if( t<r.x )
        r = vec4( t, (ob+t*rd)/rb );
    }
    return r;
}






#define Real float
#define Real3 vec3
#define fabs abs
#define INFINITY 3.402823e+38


/**
* Ray in 3-space
*/
struct Ray3 {
    Real3 origin, direction;
} ;


/**
* Cone in 3-space.
* The cone has vertex V, unit-length axis direction D, angle theta in (0,pi/2).
* Also a height and cap position are defined along the axial ray.
* The cone vertex is the ray origin and the cone axis direction is the
* ray direction. The direction must be unit length. The angle must be
* in (0,pi/2). The height must be in (0,+infinity), where +infinity is INFINITY.
*/
struct Cone3 {
    Ray3 ray;
    Real angle;
    Real height;
    Real cap; // to cap the pointy end, set this to a value in (0,height)
    Real3 heightNormal;
    Real3 capNormal;
};
    
/**
* Line in 3-space
*/
struct Line3 {
    Real3 start;
    Real3 end;
};
    
/**
*
*/
struct Plane3 {
    Real3 middle;
    Real3 normal;
};


/*
* Distance of p to plane with origin o and normalized normal n
*/
Real calcPointPlaneDistance(Real3 p, Real3 o, Real3 n) {
    Real a = dot(o, n);
    a -= dot(n, p);
    a /= dot(n, n);
    return fabs(a);
}


/*
* An intersection routine for rays and planes
*/
Real intersectPlane(Real3 pos, Real3 dir, Real3 middle, Real3 normal) {
    Real a = dot(dir, normal);
    if (a > -1e-6) return -1.0; // facing away
    Real b = dot(middle - pos, normal);
    if (b > -1e-6) return -1.0; // behind plane
    return b / a;
}


/**
* Get point rotated 90 degrees around vector (counter-clockwise).
* Basically application of rotation matrix with simplified sines and cosines.
* https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
*/
Real3 rotate90PointAroundVector(Real3 vector, Real3 point) {
    Real3 u = normalize(vector);
    return Real3(
        point.x * (u.x*u.x) + point.y * (u.x*u.y-u.z) + point.z * (u.x*u.z+u.y),
        point.x * (u.y*u.x+u.z) + point.y * (u.y*u.y) + point.z * (u.y*u.z-u.x),
        point.x * (u.z*u.x-u.y) + point.y * (u.z*u.y+u.x) + point.z * (u.z*u.z));
}


/**
* Get point on plane that minimizes distance to another point
* http://immersivemath.com/ila/ch03_dotproduct/ch03.html#ex_dp_ortho_proj_onto_plane
*/
Real3 projectPointToPlane(Plane3 plane, Real3 point) {
    Real3 v = point - plane.middle;
    Real3 proj = (dot(v, plane.normal) / pow(length(plane.normal), 2.)) * plane.normal;
    return v - proj;
}

struct ConeIntersection {
    bool exists;
    Real front;
    Real back;
    Real3 normal;
    Real3 backNormal;
};

/**
* David Eberly, Geometric Tools, Redmond WA 98052
* Copyright (c) 1998-2018
* Distributed under the Boost Software License, Version 1.0.
* http://www.boost.org/LICENSE_1_0.txt
* http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
* File Version: 3.0.2 (2018/10/05)
*
*   type  intersect  valid data
*   0     none       none
*   1     point      parameter[0] = parameter[1], finite
*                    point[0] = point[1]
*   2     segment    parameter[0] < parameter[1], finite
*                    point[0,1] valid
*   3     ray        parameter[0] finite, parameter[1] maxReal
*                    point[0] = rayOrigin, point[1] = lineDirection
*   4     ray        parameter[0] -maxReal, parameter[1] finite
*                    point[0] = rayOrigin, point[1] = -lineDirection
*   5     line       parameter[0] -maxReal, parameter[1] maxReal,
*                    point[0] = lineOrigin, point[1] = lineDirection
* If the cone height h is finite, only types 0, 1, or 2 can occur.
* https://www.geometrictools.com/Documentation/IntersectionLineCone.pdf
*/
ConeIntersection intersectCone(Real3 lineOrigin, Real3 lineDirection, Cone3 cone) {
    int type;
    float parameter0;
    float parameter1;
    // The cone has vertex V, unit-length axis direction D, angle theta in
    // (0,pi/2), and height h in (0,+infinity).  The line is P + t*U, where U
    // is a unit-length direction vector.  Define g = cos(theta).  The cone
    // is represented by
    //   (X-V)^T * (D*D^T - g^2*I) * (X-V) = 0,  0 <= dot(D,X-V) <= h
    // The first equation defines a double-sided cone.  The first inequality
    // in the second equation limits this to a single-sided cone containing
    // the ray V + s*D with s >= 0.  We will call this the 'positive cone'.
    // The single-sided cone containing ray V + s * t with s <= 0 is called
    // the 'negative cone'.  The double-sided cone is the union of the
    // positive cone and negative cone.  The second inequality in the second
    // equation limits the single-sided cone to the region bounded by the
    // height.  Setting X(t) = P + t*U, the equations are
    //   c2*t^2 + 2*c1*t + c0 = 0,  0 <= dot(D,U)*t + dot(D,P-V) <= h
    // where
    //   c2 = dot(D,U)^2 - g^2
    //   c1 = dot(D,U)*dot(D,P-V) - g^2*dot(U,P-V)
    //   c0 = dot(D,P-V)^2 - g^2*dot(P-V,P-V)
    // The following code computes the t-interval that satisfies the quadratic
    // equation subject to the linear inequality constraints.

    ConeIntersection result;
    
    Real t;

    Real3 PmV = lineOrigin - cone.ray.origin;
    Real DdU = dot(cone.ray.direction, lineDirection);
    Real DdPmV = dot(cone.ray.direction, PmV);
    Real UdPmV = dot(lineDirection, PmV);
    Real PmVdPmV = dot(PmV, PmV);
    Real cosAngle = cos(cone.angle);
    Real cosAngleSqr = cosAngle * cosAngle;
    Real c2 = DdU * DdU - cosAngleSqr;
    Real c1 = DdU * DdPmV - cosAngleSqr * UdPmV;
    Real c0 = DdPmV * DdPmV - cosAngleSqr * PmVdPmV;

    if (c2 != Real(0))
    {
        Real discr = c1 * c1 - c0 * c2;
        if (discr < Real(0))
        {
            // The quadratic has no real-valued roots.  The line does not
            // intersect the double-sided cone.
            type = 0;
            result.exists = false;
        }
        else if (discr > Real(0))
        {
            // The quadratic has two distinct real-valued roots.  However, one
            // or both of them might intersect the negative cone.  We are
            // interested only in those intersections with the positive cone.
            Real root = sqrt(discr);
            Real invC2 = (Real(1)) / c2;
            bool found1 = false;
            bool found2 = false;

            t = (-c1 - root) * invC2;
            if (DdU * t + DdPmV >= Real(0))
            {
                parameter0 = t;
                found1 = true;
            }

            t = (-c1 + root) * invC2;
            if (DdU * t + DdPmV >= Real(0))
            {
                if (!found1) parameter0 = t;
                else parameter1 = t;
                found2 = true;
            }

            if (found2)
            {
                // The line intersects the positive cone in two distinct
                // points.
                type = 2;
                if (parameter0 > parameter1)
                {
                    Real tmp = parameter0;
                    parameter0 = parameter1;
                    parameter1 = tmp;
                }
            }
            
            else if (found1)
            {
                // The line intersects the positive cone in a single point and
                // the negative cone in a single point.  We report only the
                // intersection with the positive cone.
                if (DdU > Real(0))
                {
                    // Line enters positive cone at t==parameter[0]
                    // and keeps intersecting until t==INFINITY,
                    // in case there is no cone height constraint.
                    type = 3;
                    parameter1 = INFINITY;
                }
                else
                {
                    // Line comes from t==-INFINITY and enters positive
                    // cone at t==parameter[0] (assuming no height constraint).
                    type = 4;
                    parameter1 = parameter0;
                    parameter0 = -INFINITY;
                }
            }
            else
            {
                // The line intersects the negative cone in two distinct
                // points, but we are interested only in the intersections
                // with the positive cone.
                type = 0;
                result.exists = false;
            }
        }
        else  // discr == 0
        {
            // One repeated real root; the line is tangent to the double-sided
            // cone at a single point.  Report only the point if it is on the
            // positive cone.
            t = -c1 / c2;
            if (DdU * t + DdPmV >= Real(0))
            {
                type = 1;
                parameter0 = t;
                parameter1 = t;
            }
            else
            {
                type = 0;
                result.exists = false;
            }
        }
    }
    else if (c1 != Real(0))
    {
        // c2 = 0, c1 != 0; U is a direction vector on the cone boundary
        t = -(Real(0.5))*c0 / c1;
        if (DdU * t + DdPmV >= Real(0))
        {
            // The line intersects the positive cone and the ray of
            // intersection is interior to the positive cone.
            if (DdU > Real(0))
            {
                type = 3;
                parameter0 = t;
                parameter1 = INFINITY;
            }
            else
            {
                type = 4;
                parameter0 = -INFINITY;
                parameter1 = t;
            }
        }
        else
        {
            // The line intersects the negative cone and the ray of
            // intersection is interior to the positive cone.
            type = 0;
            result.exists = false;
        }
    }
    else if (c0 != Real(0))
    {
        // c2 = c1 = 0, c0 != 0.  Cross(D,U) is perpendicular to Cross(P-V,U)
        type = 0;
        result.exists = false;
    }
    else
    {
        // c2 = c1 = c0 = 0; the line is on the cone boundary.
        type = 5;
        parameter0 = -INFINITY;
        parameter1 = +INFINITY;
    }

    // Post processing for bounded cones (cap,height)
    if (cone.height < INFINITY)
    {
        // Ignore intersections outside
        if (type > 0) {
            
            Real3 capPoint = cone.ray.origin + cone.cap * cone.ray.direction;
            Real3 heightPoint = cone.ray.origin + cone.height * cone.ray.direction;
            
            Real3 p0 = lineOrigin + parameter0 * lineDirection;
            bool p0outsideCap = dot(cone.capNormal, p0 - capPoint) > 0.0;
            bool p0outsideHeight = dot(cone.heightNormal, p0 - heightPoint) > 0.0;
            bool p0outside = p0outsideCap || p0outsideHeight;

            if (type == 1) {
                if (p0outside) type = 0;
            }
            else if (type == 2) {
                Real3 p1 = lineOrigin + parameter1 * lineDirection;
                bool p1outsideCap = dot(cone.capNormal, p1 - capPoint) > 0.0;
                bool p1outsideHeight = dot(cone.heightNormal, p1 - heightPoint) > 0.0;
                bool p1outside = p1outsideCap || p1outsideHeight;

                if (p0outside && p1outside) type = 0;
                else if (p0outside) {
                    parameter0 = parameter1;
                    type = 1;
                }
                else if (p1outside) {
                    parameter1 = parameter0;
                    type = 1;
                }
            }
        }
    }

    if (type > 0) {

        // Calc normal
        Real3 p = lineOrigin + parameter0 * lineDirection;
        Real3 grad = normalize(p - cone.ray.origin);
        Real3 normal = normalize(cross(grad, cross(grad, cone.ray.direction)));

        result.exists = true;
        result.front = parameter0;
        if (type == 2) {
            result.back = parameter1;
            p = lineOrigin + parameter1 * lineDirection;
            grad = normalize(p - cone.ray.origin);
            result.backNormal = normalize(cross(grad, cross(grad, cone.ray.direction)));
        } else result.back = -1.0;
        result.normal = normal;
        return result;
    } else {
        result.exists = false;
        return result;
    }
}

vec4 iCappedConeDE( in vec3  ro, in vec3  rd, 
                  in vec3  pb, in vec3  pa, 
                  in float rb, in float ra, vec3 na, vec3 nb, out ConeIntersection hit )
{
    if (rb < ra) {
        float tmp = ra;
        ra = rb;
        rb = tmp;
        vec3 tmp2 = pa;
        pa = pb;
        pb = tmp2;
        vec3 tmp3 = na;
        na = nb;
        nb = tmp3;
    }
    
    float slope = (rb - ra) / distance(pa, pb);
    float pzero = -ra / slope;
    vec3 coneDir = normalize(pb - pa);
    vec3 coneOrigin = pa + pzero * coneDir;
    float coneAngle = atan(slope);
    float coneCap = distance(pa, coneOrigin);
    float coneHeight = distance(pb, coneOrigin);
    Cone3 cone = Cone3(Ray3(coneOrigin, coneDir), coneAngle, coneHeight, coneCap, na, nb);

    hit = intersectCone(ro, rd, cone);
    vec4 tnor = vec4(hit.exists ? hit.front : -1, hit.normal);
    return tnor;
}


float projectC(vec3 A, vec3 B, vec3 C) {
    float Cos = dot(normalize(C-A), normalize(B-A));
    float c = distance(A,C);
    float x = Cos * c;
    return x / distance(A,B);
}

vec4 getColor(vec3 pos, vec3 normal, vec3 viewDir) {
    vec4 c0 = color0_;
    vec4 c1 = color1_;
    vec4 color = c0 + projectC(startPos_, endPos_, pos) * (c1 - c0);
    vec3 rgb = APPLY_LIGHTING(lighting, color.rgb, color.rgb, vec3(1.0f), pos, normal, viewDir);
    return vec4(rgb, color.a);
}

void main() {

    if (startPos_ == endPos_) discard;

    vec3 start = startPos_;
    vec3 end = endPos_;
    float r0 = radius0_;
    float r1 = radius1_;

    vec3 camPos = (camera.viewToWorld * vec4(0,0,0,1)).xyz;
    vec3 dir = normalize(worldPos_ - camPos);

    ConeIntersection hit = ConeIntersection(false,-1,-1,vec3(0), vec3(0));
    vec4 tnor = iCappedConeDE(camPos, dir, start, end, r0, r1, -gEndplanes[0], gEndplanes[1], hit);
    //vec4 tnor = iRoundedCone(worldPos_, dir, start, end, r0, r1);
    //vec4 tnor = coneIntersect(worldPos_, dir, start, end, r0, r1);

    //FIXME Viewing endplanes head-on causes very close hits (t < 0.0001) to be detected.
    //      Are the coordinates on the endplanes corrupted
    // It is related to the artificial scaling of the box.
    // And it seems to be the endplane in the back => Cull backfaces?

    if (tnor.x < 0.0) {
        discard;
        return;
    }

    vec3 hitPoint = camPos + tnor.x * dir;
    vec4 color = getColor(hitPoint, tnor.yzw, -dir);
    vec4 ndc = camera.worldToClip * vec4(hitPoint, 1);
    float depth = ((ndc.z / ndc.w) + 1 ) / 2;

#if defined(USE_FRAGMENT_LIST)
    if (color.a > 0.0) {
        abufferRender(ivec2(gl_FragCoord.xy), depth, color);

        if (hit.back >= 0.0) {
            vec3 hitPointBack = camPos + hit.back * dir;
            vec4 colorBack = getColor(hitPointBack, hit.backNormal, -dir);
            vec4 ndcBack = camera.worldToClip * vec4(hitPointBack, 1);
            float depthBack = ((ndcBack.z / ndcBack.w) + 1 ) / 2;
            abufferRender(ivec2(gl_FragCoord.xy), depthBack, colorBack);
        }
    }
    discard;

#else  // USE_FRAGMENT_LIST

    gl_FragDepth = depth;
    FragData0 = color;
    PickingData = pickColor_;
    
#endif // not USE_FRAGMENT_LIST
}