/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2015-2020 Inviwo Foundation
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

#ifndef IVW_ANSYSFIELDREADER3D_H
#define IVW_ANSYSFIELDREADER3D_H

#include <modules/vectorfieldvisualizationgl/vectorfieldvisualizationglmoduledefine.h>
#include <inviwo/core/common/inviwo.h>

#include <inviwo/core/ports/meshport.h>
#include <inviwo/core/ports/imageport.h>
#include <modules/vectorfieldvisualization/ports/seedpointsport.h>
#include <inviwo/core/ports/dataoutport.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/stringproperty.h>
#include <inviwo/core/properties/fileproperty.h>
#include <inviwo/core/properties/stringproperty.h>
#include <inviwo/core/properties/buttonproperty.h>
#include <inviwo/core/properties/minmaxproperty.h>
#include <inviwo/core/properties/compositeproperty.h>
#include <inviwo/core/properties/boolcompositeproperty.h>
#include <inviwo/core/properties/optionproperty.h>
#include <inviwo/core/properties/eventproperty.h>
#include <inviwo/core/properties/cameraproperty.h>

#include <inviwo/core/datastructures/image/imageram.h>
#include <inviwo/core/datastructures/image/layerramprecision.h>
#include <inviwo/core/datastructures/spatialdata.h>
#include <inviwo/core/util/spatialsampler.h>
#include <modules/base/algorithm/meshutils.h>
#include <modules/base/datastructures/kdtree.h>

namespace SimData3D {
struct Subgroup {
    size_t start, end;
    int id;
};

struct PointCloud : public inviwo::SpatialEntity<3> {
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> vel;
    std::vector<float> velmag;
    std::vector<float> dist;
    std::vector<float> WSS;
    std::vector<float> press;
    std::vector<float> vorticity;
    std::vector<float> boundarydist;
    std::vector<Subgroup> subgroups;
    PointCloud() {}
    PointCloud(const PointCloud& toCopy)
        : points(toCopy.points)
        , vel(toCopy.vel)
        , velmag(toCopy.velmag)
        , dist(toCopy.dist)
        , WSS(toCopy.WSS)
        , press(toCopy.press)
        , vorticity(toCopy.vorticity)
        , boundarydist(toCopy.boundarydist)
        , subgroups(toCopy.subgroups) {}
    PointCloud* clone() const { return new PointCloud(*this); }
};
}  // namespace SimData3D

using Real = double;
using Real3 = glm::dvec3;

Real intersectPlaneBidirectional(Real3 pos, Real3 dir, Real3 middle, Real3 normal);

namespace inviwo {

namespace SimDataSampling3D {

struct AABB {
    glm::dvec3 min, max;
    AABB() : min(0), max(0) {}
    AABB(std::vector<glm::vec3> points)
        : min{std::numeric_limits<float>::infinity()}
        , max{-std::numeric_limits<float>::infinity()} {
        for (const auto node : points) {
            if (node.x < min.x) min.x = node.x;
            if (node.y < min.y) min.y = node.y;
            if (node.z < min.z) min.z = node.z;
            if (node.x > max.x) max.x = node.x;
            if (node.y > max.y) max.y = node.y;
            if (node.z > max.z) max.z = node.z;
        }
    }
    AABB(const Mesh& mesh)
        : min{std::numeric_limits<float>::infinity()}
        , max{-std::numeric_limits<float>::infinity()} {
        const auto buffers = mesh.getBuffers();
        for (const auto buf : buffers) {
            if (buf.first.type == BufferType::PositionAttrib) {
                const auto posBuf = buf.second->getRepresentation<BufferRAM>();
                for (size_t i = 0; i < posBuf->getSize(); i++) {
                    const auto val = posBuf->getAsDVec3(i);
                    if (val.x < min.x) min.x = val.x;
                    if (val.y < min.y) min.y = val.y;
                    if (val.z < min.z) min.z = val.z;
                    if (val.x > max.x) max.x = val.x;
                    if (val.y > max.y) max.y = val.y;
                    if (val.z > max.z) max.z = val.z;
                }
                break;
            }
        }
    }
};

using namespace SimData3D;

struct PointCloudVelocitySampler : public SpatialSampler<3, 3, double> {

    const Mesh& boundaries;
    const SimData3D::PointCloud& pointcloud;
    K3DTree<int, float> kdtree;
    AABB pointCloudAABB;
    AABB meshAABB;
    vec3 meshSizeFactor;

    PointCloudVelocitySampler(SimData3D::PointCloud& pointcloud, const Mesh& boundaries,
                              bool rescale = true)
        : SpatialSampler<3, 3, double>(pointcloud)
        , boundaries(boundaries)
        , pointcloud(pointcloud)
        , kdtree()
        , pointCloudAABB(pointcloud.points)
        , meshAABB(boundaries)
        , meshSizeFactor(1) {

        const auto meshWidth = meshAABB.max.x - meshAABB.min.x;
        const auto meshHeight = meshAABB.max.y - meshAABB.min.y;
        const auto meshDepth = meshAABB.max.z - meshAABB.min.z;

        const auto pointcloudWidth = pointCloudAABB.max.x - pointCloudAABB.min.x;
        const auto pointcloudHeight = pointCloudAABB.max.y - pointCloudAABB.min.y;
        const auto pointcloudDepth = pointCloudAABB.max.z - pointCloudAABB.min.z;

        if (rescale) {

            meshSizeFactor = vec3(meshWidth / pointcloudWidth, meshHeight / pointcloudHeight,
                                  meshDepth / pointcloudDepth);
        }

        // pointcloud.setModelMatrix(glm::scale(vec3(meshSizeFactor, 1)));

        for (size_t i = 0; i < pointcloud.points.size(); i++) {
            if (rescale) pointcloud.points[i] = pointcloud.points[i] * meshSizeFactor;

            kdtree.insert(pointcloud.points[i], (int)i);
        }
    }

    bool debugTriangles(const dvec3& pos, std::vector<vec3>& verts,
                        std::vector<vec3>& norms) const {

        const auto buffers = boundaries.getBuffers();
        const BufferRAM* vertices = 0;
        const BufferRAM* normals = 0;
        for (const auto buf : buffers) {
            if (buf.first.type == BufferType::PositionAttrib) {
                vertices = buf.second->getRepresentation<BufferRAM>();
            }
            if (buf.first.type == BufferType::NormalAttrib) {
                normals = buf.second->getRepresentation<BufferRAM>();
            }
        }

        bool result = false;

        if (vertices != 0 && normals != 0) {

            const auto r0 = dvec3(this->meshAABB.min) - dvec3(0.5);
            const auto r1 = dvec3(pos);

            const auto ray = dvec3(r0);
            const auto dir = dvec3(glm::normalize(r1 - r0));

            int isectCount = 0;

            for (auto& ib : boundaries.getIndexBuffers()) {
                if (ib.first.dt == DrawType::Triangles) {
                    meshutil::forEachTriangle(ib.first, *ib.second,
                                              [&](uint32_t i0, uint32_t i1, uint32_t i2) {
                                                  verts.push_back(vertices->getAsDVec3(i0));
                                                  verts.push_back(vertices->getAsDVec3(i1));
                                                  verts.push_back(vertices->getAsDVec3(i2));
                                                  norms.push_back(normals->getAsDVec3(i0));
                                                  norms.push_back(normals->getAsDVec3(i1));
                                                  norms.push_back(normals->getAsDVec3(i2));
                                              });
                }
            }

            result = isectCount % 2 != 0;
        }

        return result;
    }

    bool inside(const dvec3& pos, const dvec3 rayStartOffset = dvec3(0.5)) const {

        const auto buffers = boundaries.getBuffers();
        const BufferRAM* vertices = 0;
        const BufferRAM* normals = 0;
        for (const auto buf : buffers) {
            if (buf.first.type == BufferType::PositionAttrib) {
                vertices = buf.second->getRepresentation<BufferRAM>();
            }
            if (buf.first.type == BufferType::NormalAttrib) {
                normals = buf.second->getRepresentation<BufferRAM>();
            }
        }

        bool result = false;

        if (vertices != 0 && normals != 0) {

            const auto r0 = this->meshAABB.min - rayStartOffset;
            const auto r1 = pos;

            const auto ray = r0;
            const auto dir = glm::normalize(r1 - r0);

            int isectCount = 0;

            for (auto& ib : boundaries.getIndexBuffers()) {
                if (ib.first.dt == DrawType::Triangles) {
                    meshutil::forEachTriangle(
                        ib.first, *ib.second, [&](uint32_t i0, uint32_t i1, uint32_t i2) {
                            const auto v0 = vertices->getAsDVec3(i0);
                            const auto v1 = vertices->getAsDVec3(i1);
                            const auto v2 = vertices->getAsDVec3(i2);
                            const auto e0 = v1 - v0;
                            const auto e1 = v2 - v1;
                            const auto e2 = v0 - v2;

                            auto N = glm::normalize(glm::cross(e0, e1));
                            auto t = intersectPlaneBidirectional(ray, dir, v0, N);
                            if (t != std::numeric_limits<Real>::infinity() &&
                                t <= glm::distance(r0, r1)) {
                                auto P = ray + t * dir;
                                const auto C0 = P - v0;
                                const auto C1 = P - v1;
                                const auto C2 = P - v2;
                                if ((glm::dot(N, glm::cross(e0, C0)) > 0) &&
                                    (glm::dot(N, glm::cross(e1, C1)) > 0) &&
                                    (glm::dot(N, glm::cross(e2, C2))) > 0) {
                                    isectCount++;
                                }
                            }
                        });
                }
            }

            result = isectCount % 2 != 0;
        }

        return result;
    }

    virtual Vector<3, double> sampleDataSpace(const dvec3& pos) const {

        if (!inside(pos)) return dvec3(0);

        return pointcloud.vel[*kdtree.findNearest(pos)->getDataAsPointer()];

        /*size_t nearestNeighbor = 0;
        float smallestDistance = std::numeric_limits<float>::infinity();
        for (size_t k = 0; k < pointcloud.points.size(); k++) {
            if (distance(pointcloud.points[k], vec3(pos)) < smallestDistance) {
                nearestNeighbor = k;
                smallestDistance = distance(pointcloud.points[nearestNeighbor], vec3(pos));
            }
        }
        return pointcloud.vel[nearestNeighbor];*/
    }

    virtual bool withinBoundsDataSpace(const dvec3& pos) const {
        return inside(pos);
        return !(glm::any(glm::lessThan(pos, dvec3(meshAABB.min))) ||
                 glm::any(glm::greaterThan(pos, dvec3(meshAABB.max))));
    }
};

struct PointCloudScalarSampler : public SpatialSampler<3, 1, double> {

    const Mesh& boundaries;
    const SimData3D::PointCloud& pointcloud;
    std::vector<float>& scalars;
    K3DTree<int, float> kdtree;
    AABB pointCloudAABB;
    AABB meshAABB;
    vec3 meshSizeFactor;

    PointCloudScalarSampler(SimData3D::PointCloud& pointcloud, std::vector<float>& scalars,
                            const Mesh& boundaries)
        : SpatialSampler<3, 1, double>(pointcloud)
        , boundaries(boundaries)
        , pointcloud(pointcloud)
        , scalars(scalars)
        , kdtree()
        , pointCloudAABB(pointcloud.points)
        , meshAABB(boundaries)
        , meshSizeFactor(1) {

        for (size_t i = 0; i < pointcloud.points.size(); i++) {

            kdtree.insert(pointcloud.points[i], (int)i);
        }
    }

    virtual double sampleDataSpace(const dvec3& pos) const {

        if (scalars.empty()) return .0;
        auto nearest = kdtree.findNearest(pos);
        auto ptr = nearest->getDataAsPointer();
        return scalars[*ptr];

        /*size_t nearestNeighbor = 0;
        float smallestDistance = std::numeric_limits<float>::infinity();
        for (size_t k = 0; k < pointcloud.points.size(); k++) {
            if (distance(pointcloud.points[k], vec3(pos)) < smallestDistance) {
                nearestNeighbor = k;
                smallestDistance = distance(pointcloud.points[nearestNeighbor], vec3(pos));
            }
        }
        return pointcloud.vel[nearestNeighbor];*/
    }

    virtual bool withinBoundsDataSpace(const dvec3& pos) const {
        return !(glm::any(glm::lessThan(pos, dvec3(meshAABB.min))) ||
                 glm::any(glm::greaterThan(pos, dvec3(meshAABB.max))));
    }
};

}  // namespace SimDataSampling3D

class IVW_MODULE_VECTORFIELDVISUALIZATIONGL_API AnsysFieldReader3D : public Processor {
public:
    AnsysFieldReader3D();
    virtual ~AnsysFieldReader3D();

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

    virtual void initializeResources() override;
    virtual void process() override;

protected:
    enum class Scalar {
        Vorticity,
        Pressure,
        WSS,
        CellWallDistance,
        BoundaryDistance,
        VelocityMagnitude
    };

    MeshInport boundaries_;
    MeshInport seedSurface_;

    MeshOutport pointCloud_;
    DataOutport<SpatialSampler<3, 3, double>> sampler_;
    DataOutport<SpatialSampler<3, 1, double>> scalarSampler_;
    MeshOutport insideTest_;
    SeedPoints3DOutport streamlineSeeds_;
    DataOutport<std::vector<vec3>> points_;
    DataOutport<std::vector<vec3>> vel_;

    FileProperty file_;
    BoolProperty rescale_;
    BoolProperty noisify_;
    FloatProperty noise_;
    ButtonProperty readButton_;
    ButtonProperty seedButton_;
    IntSizeTProperty numSeeds_;
    TemplateOptionProperty<Scalar> scalar_;

    OptionPropertyInt subgroupSelector_;

    CompositeProperty pointcloudVis_;
    FloatProperty pointSize_;
    FloatProperty velocityScaling_;

    CompositeProperty spaceStats_;
    IntSizeTProperty pointCloudSize_;
    ButtonProperty computeStepButton_;
    FloatProperty pointCloudMinNearestDistance_;
    FloatProperty pointCloudMaxNearestDistance_;
    FloatProperty pointCloudAvgNearestDistance_;
    FloatVec3Property pointCloudMin_;
    FloatVec3Property pointCloudMax_;
    FloatVec3Property meshMin_;
    FloatVec3Property meshMax_;

    CompositeProperty interactive_;
    BoolProperty enableInteractive_;
    EventProperty hoverEvents_;
    EventProperty clickEvents_;
    FloatVec3Property insideTestPoint_;

    SimData3D::PointCloud pointcloud;
    std::shared_ptr<SimDataSampling3D::PointCloudVelocitySampler> velocitySampler;

    void createSampler(const Mesh& boundaries);

    void seedOnInputSurface();

    std::shared_ptr<Mesh> pointCloudToMesh3D(SimData3D::PointCloud& pointcloud);

    void processPickEvent(Event* e);
};

}  // namespace inviwo

#endif  // IVW_ANSYSFIELDREADER3D_H
