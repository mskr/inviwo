/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2016-2020 Inviwo Foundation
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

#ifndef IVW_INTEGRALLINECOMPARE_H
#define IVW_INTEGRALLINECOMPARE_H

#include <modules/vectorfieldvisualization/vectorfieldvisualizationmoduledefine.h>

#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/ports/dataoutport.h>
#include <inviwo/core/ports/meshport.h>

#include <inviwo/core/properties/stringproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>

#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/boolcompositeproperty.h>
#include <inviwo/core/properties/buttonproperty.h>
#include <inviwo/core/properties/minmaxproperty.h>
#include <inviwo/core/properties/optionproperty.h>
#include <inviwo/core/properties/compositeproperty.h>
#include <inviwo/core/datastructures/transferfunction.h>
#include <inviwo/core/properties/transferfunctionproperty.h>

#include <modules/vectorfieldvisualization/datastructures/integralline.h>
#include <modules/vectorfieldvisualization/datastructures/integrallineset.h>
#include <inviwo/core/datastructures/geometry/typedmesh.h>
#include <inviwo/core/datastructures/geometry/basicmesh.h>

namespace inviwo {

struct LinePair;

struct MeanLine {
    IntegralLine line;
    std::vector<float> deviations;
    size_t endIndex;
    LinePair* pair;
    bool isPairDiverged = false;
    bool isPairOfDifferentLength = false;
};

struct BridgeFace {
    std::vector<vec3> l1Normals;
    std::vector<vec3> l2Normals;
};

struct LinePair {
    IntegralLine l1, l2;
    LinePair(IntegralLine l1, IntegralLine l2) : l1(l1), l2(l2) {}
    bool bothExist() { return !l1.getPositions().empty() && !l2.getPositions().empty(); }
    LinePair lastPart(size_t from) {
        IntegralLine result1;
        IntegralLine result2;
        auto size = std::max(l1.getPositions().size(), l2.getPositions().size());
        for (size_t i = from; i < size; i++) {
            if (i < l1.getPositions().size())
                result1.getPositions().emplace_back(l1.getPositions()[i]);
            if (i < l2.getPositions().size())
                result2.getPositions().emplace_back(l2.getPositions()[i]);
        }
        return LinePair(result1, result2);
    }
    MeanLine meanLineUntil(float splitThreshold) {
        MeanLine result;
        auto p1 = l1.getPositions();
        auto p2 = l2.getPositions();
        auto size = std::min(p1.size(), p2.size());
        size_t i = 0;
        for (; i < size; i++) {
            dvec3 mean = (p1[i] + p2[i]) / 2.0;
            float d = static_cast<float>(glm::distance(p1[i], mean));

            if (d > splitThreshold) {
                result.isPairDiverged = true;

                if (i > 0) {
                    dvec3 prevMean = (p1[i - 1] + p2[i - 1]) / 2.0;
                    float d_prev = static_cast<float>(glm::distance(p1[i - 1], prevMean));
                    auto interpolatedMean =
                        prevMean + static_cast<double>((splitThreshold - d_prev) / (d - d_prev)) *
                                       (mean - prevMean);

                    result.line.getPositions().emplace_back(interpolatedMean);
                    result.line.getMetaData<dvec3>("velocity", true)
                        .push_back((l1.getMetaData<dvec3>("velocity", true)[i] +
                                    l2.getMetaData<dvec3>("velocity", true)[i]) /
                                   2.0);
                    result.deviations.push_back(splitThreshold);
                }

                break;
            }

            result.line.getPositions().emplace_back(mean);
            result.line.getMetaData<dvec3>("velocity", true)
                .push_back((l1.getMetaData<dvec3>("velocity", true)[i] +
                            l2.getMetaData<dvec3>("velocity", true)[i]) /
                           2.0);
            result.deviations.push_back(d);
        }

        if (p1.size() != p2.size()) {
            result.isPairOfDifferentLength = true;
        }

        result.endIndex = i;
        result.pair = this;
        return result;
    }
    BridgeFace bridgeFace() {
        BridgeFace result;
        auto p1 = l1.getPositions();
        auto p2 = l2.getPositions();
        if (p1.empty() || p2.empty()) return result;

        auto size = std::max(p1.size(), p2.size());

        vec3 lastL1BridgeVec, lastL2BridgeVec;

        for (size_t i = 0; i < size; i++) {
            if (i < p1.size()) {
                vec3 l1BridgeVec;
                vec3 l1Dir;
                if (i < p2.size())
                    l1BridgeVec = p2[i] - p1[i];
                else
                    l1BridgeVec = lastL1BridgeVec;
                if (i == 0 && i < p1.size() - 1)
                    l1Dir = p1[i + 1] - p1[i];
                else if (i < p1.size() - 1)
                    l1Dir = p1[i + 1] - p1[i - 1];
                else
                    l1Dir = p1[i] - p1[i - 1];
                vec3 l1BridgeNormal =
                    glm::cross(glm::normalize(l1BridgeVec), glm::normalize(l1Dir));
                result.l1Normals.push_back(glm::cross(glm::normalize(l1Dir), l1BridgeNormal));
                lastL1BridgeVec = l1BridgeVec;
            }

            if (i < p2.size()) {
                vec3 l2BridgeVec;
                if (i < p1.size())
                    l2BridgeVec = p1[i] - p2[i];
                else
                    l2BridgeVec = lastL2BridgeVec;
                vec3 l2Dir;
                if (i == 0 && i < p2.size() - 1)
                    l2Dir = p2[i + 1] - p2[i];
                else if (i < p2.size() - 1)
                    l2Dir = p2[i + 1] - p2[i - 1];
                else
                    l2Dir = p2[i] - p2[i - 1];
                vec3 l2BridgeNormal =
                    glm::cross(glm::normalize(l2BridgeVec), glm::normalize(l2Dir));
                result.l2Normals.push_back(glm::cross(glm::normalize(l2Dir), l2BridgeNormal));
                lastL2BridgeVec = l2BridgeVec;
            }
        }
        return result;
    }
};

class IVW_MODULE_VECTORFIELDVISUALIZATION_API IntegralLineCompare : public Processor {
public:
    IntegralLineCompare();
    virtual ~IntegralLineCompare() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    IntegralLineSetInport lines1_;
    IntegralLineSetInport lines2_;
    IntegralLineSetOutport out_;
    MeshOutport tubeMesh_;
    MeshOutport podMesh_;
    DataOutport<std::vector<vec4>> colors_;

    enum class SeverenessMetric { Runlength, RunlengthUntilDiverged, ThicknessSum, LengthAndThickness };

    FloatProperty matchTolerance_;
    BoolProperty noTriples_;
    BoolProperty tubes_;
    FloatProperty splitThreshold_;

    CompositeProperty diverged_;
    FloatProperty podSize_;
    FloatVec4Property podColor_;
    FloatVec4Property earlyEndColor_;
    FloatProperty divergedLines_;

    CompositeProperty severity_;
    TemplateOptionProperty<SeverenessMetric> severityMetric_;
    IntSizeTProperty severityFilter_;

    CompositeProperty colorMaps_;
    TransferFunctionProperty thickness_;
    BoolProperty enableRunlength_;
    TransferFunctionProperty runlength_;
    BoolProperty enableWallDistance_;
    TransferFunctionProperty wallDistance_;

    float severeness(MeanLine line);

    struct BoundedFloat {
        float v, min, max;
    };

    vec4 colorMapping(BoundedFloat thickness, BoundedFloat runlength,
                      BoundedFloat wallDistance = {0.f, 0.f, 0.f});

    using MyLineMesh =
        TypedMesh<buffertraits::PositionsBuffer, buffertraits::RadiiBuffer,
                  buffertraits::NormalBuffer, buffertraits::ColorsBuffer,
                  buffertraits::TexcoordBuffer<3>>;

    void tubeGfx(std::shared_ptr<MyLineMesh> mesh, std::vector<dvec3> vertices,
                 std::function<float(int)> radius, std::function<vec4(int)> color,
                 std::function<vec3(int)> halftubeNormal);

    void podGfx(std::shared_ptr<BasicMesh> mesh, vec3 pos, float radius, vec4 color);

};

}  // namespace inviwo

#endif  // IVW_INTEGRALLINECOMPARE_H
