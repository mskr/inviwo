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

#ifndef IVW_ANSYSFIELDREADER2D_H
#define IVW_ANSYSFIELDREADER2D_H

#include <modules/vectorfieldvisualizationgl/vectorfieldvisualizationglmoduledefine.h>
#include <inviwo/core/common/inviwo.h>

#include <inviwo/core/ports/meshport.h>
#include <inviwo/core/ports/imageport.h>
#include <modules/vectorfieldvisualization/ports/seedpointsport.h>
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

#include <inviwo/core/datastructures/image/imageram.h>
#include <inviwo/core/datastructures/image/layerramprecision.h>
#include <inviwo/core/datastructures/spatialdata.h>
#include <inviwo/core/util/spatialsampler.h>
#include <modules/base/algorithm/meshutils.h>
#include <modules/base/datastructures/kdtree.h>

namespace SimData2D {
struct Subgroup {
    size_t start, end;
    int id;
};

struct PointCloud : public inviwo::SpatialEntity<2> {
    std::vector<glm::vec2> points;
    std::vector<glm::vec2> vel;
    std::vector<float> dist;
    std::vector<float> WSS;
    std::vector<float> press;
    std::vector<Subgroup> subgroups;
    PointCloud() {}
    PointCloud(const PointCloud& toCopy)
        : points(toCopy.points)
        , vel(toCopy.vel)
        , dist(toCopy.dist)
        , WSS(toCopy.WSS)
        , press(toCopy.press)
        , subgroups(toCopy.subgroups) {}
    PointCloud* clone() const { return new PointCloud(*this); }
};
}  // namespace SimData2D

using Real = double;
using Real3 = glm::dvec3;

Real intersectPlane(Real3 pos, Real3 dir, Real3 middle, Real3 normal);

namespace inviwo {

struct AABB {
    glm::dvec2 min, max;
    AABB() : min(0), max(0) {}
    AABB(std::vector<glm::vec2> points)
        : min{std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()}
        , max{-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()} {
        for (const auto node : points) {
            if (node.x < min.x) min.x = node.x;
            if (node.y < min.y) min.y = node.y;
            if (node.x > max.x) max.x = node.x;
            if (node.y > max.y) max.y = node.y;
        }
    }
    AABB(const Mesh& mesh)
        : min{std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()}
        , max{-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()} {
        const auto buffers = mesh.getBuffers();
        for (const auto buf : buffers) {
            if (buf.first.type == BufferType::PositionAttrib) {
                const auto posBuf = buf.second->getRepresentation<BufferRAM>();
                for (size_t i = 0; i < posBuf->getSize(); i++) {
                    const auto val = posBuf->getAsDVec2(i);
                    if (val.x < min.x) min.x = val.x;
                    if (val.y < min.y) min.y = val.y;
                    if (val.x > max.x) max.x = val.x;
                    if (val.y > max.y) max.y = val.y;
                }
                break;
            }
        }
    }
};

using namespace SimData2D;

struct PointCloudVelocitySampler : public SpatialSampler<2, 2, double> {

    const Mesh& boundaries;
    const PointCloud& pointcloud;
    K2DTree<int, float> kdtree;
    AABB pointCloudAABB;
    AABB meshAABB;
    dvec2 meshSizeFactor;

    PointCloudVelocitySampler(PointCloud& pointcloud, const Mesh& boundaries)
        : SpatialSampler<2, 2, double>(pointcloud)
        , boundaries(boundaries)
        , pointcloud(pointcloud)
        , kdtree()
        , pointCloudAABB(pointcloud.points)
        , meshAABB(boundaries)
        , meshSizeFactor(1) {

        const auto meshWidth = meshAABB.max.x - meshAABB.min.x;
        const auto meshHeight = meshAABB.max.y - meshAABB.min.y;

        const auto pointcloudWidth = pointCloudAABB.max.x - pointCloudAABB.min.x;
        const auto pointcloudHeight = pointCloudAABB.max.y - pointCloudAABB.min.y;

        meshSizeFactor = dvec2(meshWidth / pointcloudWidth, meshHeight / pointcloudHeight);

        // pointcloud.setModelMatrix(glm::scale(vec3(meshSizeFactor, 1)));

        for (size_t i = 0; i < pointcloud.points.size(); i++) {
            pointcloud.points[i] = pointcloud.points[i] * vec2(meshSizeFactor);

            kdtree.insert(pointcloud.points[i], (int)i);
        }
    }

    bool inside(const dvec2& pos) const {

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

            const auto r0 = dvec2(this->meshAABB.min) - dvec2(0.5, -1.0);
            const auto r1 = dvec2(pos);

            const auto ray = dvec3(r0, 0);
            const auto dir = dvec3(glm::normalize(r1 - r0), 0);

            int isectCount = 0;

            for (size_t i = 0; i < vertices->getSize(); i += 2) {
                const auto v0 = dvec2(vertices->getAsDVec3(i));
                const auto v1 = dvec2(vertices->getAsDVec3(i + 1));

                const auto normal = normals->getAsDVec3(i);
                const auto middle = dvec3(v0, 0);

                auto t = intersectPlane(ray, dir, middle, normal);
                if (t < 0) t = intersectPlane(ray, dir, middle, -normal);
                if (t < 0) continue;

                const auto isect = ray + t * dir;

                if (isect.x >= std::min(v0.x, v1.x) && isect.x <= std::max(v0.x, v1.x) &&
                    isect.y >= std::min(v0.y, v1.y) && isect.y <= std::max(v0.y, v1.y) &&
                    isect.x >= std::min(r0.x, r1.x) && isect.x <= std::max(r0.x, r1.x) &&
                    isect.y >= std::min(r0.y, r1.y) && isect.y <= std::max(r0.y, r1.y)) {
                    isectCount++;
                }
            }

            result = isectCount % 2 != 0;
        }

        return result;
    }

    virtual Vector<2, double> sampleDataSpace(const dvec2& pos) const {

        if (!inside(pos)) return dvec2(0);

        return pointcloud.vel[*kdtree.findNearest(pos)->getDataAsPointer()];

        size_t nearestNeighbor = 0;
        float smallestDistance = std::numeric_limits<float>::infinity();
        for (size_t k = 0; k < pointcloud.points.size(); k++) {
            if (distance((dvec2(pointcloud.points[k])), pos) < smallestDistance) {
                nearestNeighbor = k;
                smallestDistance = distance((dvec2(pointcloud.points[nearestNeighbor])), pos);
            }
        }
        return dvec2(pointcloud.vel[nearestNeighbor]);
    }

    virtual bool withinBoundsDataSpace(const dvec2& pos) const {
        return inside(pos);
        return !(glm::any(glm::lessThan(pos, dvec2(meshAABB.min))) ||
                 glm::any(glm::greaterThan(pos, dvec2(meshAABB.max))));
    }
};

class IVW_MODULE_VECTORFIELDVISUALIZATIONGL_API AnsysFieldReader2D : public Processor {
public:
    AnsysFieldReader2D();
    virtual ~AnsysFieldReader2D();

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

    virtual void initializeResources() override;
    virtual void process() override;

protected:
    MeshInport boundaries_;

    MeshOutport pointCloud_;
    DataOutport<SpatialSampler<2, 2, double>> sampler_;
    ImageOutport velocityField_;
    ImageOutport wallDistanceField_;
    ImageOutport wssField_;
    ImageOutport pressField_;
    MeshOutport insideTest_;
    SeedPoints2DOutport streamlineSeeds_;

    FileProperty file_;
    ButtonProperty readButton_;

    OptionPropertyInt subgroupSelector_;

    CompositeProperty pointcloudVis_;
    FloatProperty pointSize_;
    FloatProperty velocityScaling_;

    CompositeProperty spaceStats_;
    IntSizeTProperty pointCloudSize_;
    FloatProperty pointCloudMinNearestDistance_;
    FloatProperty pointCloudMaxNearestDistance_;
    FloatProperty pointCloudAvgNearestDistance_;
    FloatVec2Property pointCloudMin_;
    FloatVec2Property pointCloudMax_;
    FloatVec2Property meshMin_;
    FloatVec2Property meshMax_;

    BoolCompositeProperty resampling_;
    IntSize2Property size_;
    FloatProperty distanceTolerance_;
    FloatProperty minDistance_;
    FloatProperty maxDistance_;
    FloatProperty avgDistance_;

    FloatVec2Property insideTestPoint_;

    PointCloud pointcloud;
    std::shared_ptr<PointCloudVelocitySampler> velocitySampler;

    template <typename Format>
    std::shared_ptr<Image> pointCloudToImage2D(
        PointCloud& pointcloud, std::function<void(LayerRAM*, size_t, size_t, size_t)> fillFn);

    std::shared_ptr<Mesh> pointCloudToMesh2D(PointCloud& pointcloud);
};

}  // namespace inviwo

#endif  // IVW_VECTORFIELDGENERATOR2D_H
