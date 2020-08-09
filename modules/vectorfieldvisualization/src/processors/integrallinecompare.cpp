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

#include <modules/vectorfieldvisualization/processors/integrallinecompare.h>
#include <modules/vectorfieldvisualization/processors/3d/pathlines.h>
#include <modules/vectorfieldvisualization/processors/3d/streamlines.h>
#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <inviwo/core/network/networklock.h>
#include <inviwo/core/util/zip.h>
#include <inviwo/core/util/raiiutils.h>
#include <modules/vectorfieldvisualization/algorithms/integrallineoperations.h>

namespace inviwo {

const ProcessorInfo IntegralLineCompare::getProcessorInfo() const { return processorInfo_; }

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo IntegralLineCompare::processorInfo_{
    "org.inviwo.IntegralLineCompare",  // Class identifier
    "Integral Line Compare",           // Display name
    "Vector Field Visualization",      // Category
    CodeState::Stable,                 // Code state
    Tags::CPU,                         // Tags
};

IntegralLineCompare::IntegralLineCompare()
    : Processor()
    , lines1_("lines1")
    , lines2_("lines2")
    , out_("out")
    , tubeMesh_("tubeMesh")
    , colors_("colors")
    , matchTolerance_("matchTolerance", "Match Tolerance", 0.3f, 0.0f, 0.3f)
    , noTriples_("noTriples", "No Triples")
    , tubes_("tubes", "Tubes")
    , splitThreshold_("splitThreshold", "Split Threshold")
    , divergedLines_("divergedLines", "Diverged Lines")
    , severenessMetric_("severenessMetric", "Severeness Metric")
    , severenessFilter_("severenessFilter", "Severeness Filter", 1000, 1, 1000)
    , colorMaps_("colorMaps", "Colormaps")
    , thickness_("thickness", "Thickness",
                 TransferFunction({{0.0f, vec4(0.0f, 0.1f, 1.0f, 1.0f)},
                                   {1.0f, vec4(1.0f, 0.03f, 0.03f, 1.0f)}}))
    , enableRunlength_("enableRunlength", "Enable Runlength")
    , runlength_("runlength", "Runlength",
                 TransferFunction({{0.0f, vec4(0.0f, 0.1f, 1.0f, 1.0f)},
                                   {1.0f, vec4(1.0f, 0.03f, 0.03f, 1.0f)}}))
    , enableWallDistance_("enableWallDistance", "Enable Wall Distance")
    , wallDistance_("wallDistance", "Wall Distance",
                    TransferFunction({{0.0f, vec4(0.0f, 0.1f, 1.0f, 1.0f)},
                                      {1.0f, vec4(1.0f, 0.03f, 0.03f, 1.0f)}})) {
    addPort(lines1_);
    addPort(lines2_);
    addPort(out_);
    addPort(tubeMesh_);
    addPort(colors_);

    severenessMetric_.addOption("runlength", "Runlength", SeverenessMetric::Runlength);
    severenessMetric_.addOption("thickness", "Thickness", SeverenessMetric::ThicknessSum);

    addProperties(matchTolerance_, noTriples_, tubes_, splitThreshold_, divergedLines_,
                  severenessMetric_, severenessFilter_, colorMaps_);

    colorMaps_.addProperties(thickness_, enableRunlength_, runlength_, enableWallDistance_,
                             wallDistance_);

    runlength_.setReadOnly(!enableRunlength_);
    wallDistance_.setReadOnly(!enableWallDistance_);
    enableRunlength_.onChange([&]() { runlength_.setReadOnly(!enableRunlength_); });
    enableWallDistance_.onChange([&]() { wallDistance_.setReadOnly(!enableWallDistance_); });
}

void IntegralLineCompare::process() {
    const auto set1 = *lines1_.getData();
    const auto set2 = *lines2_.getData();
    const auto resultSet = std::make_shared<IntegralLineSet>(mat4(1));
    const auto colors = std::make_shared<std::vector<vec4>>();
    auto mesh = std::make_shared<MyLineMesh>();

    auto distance = [](IntegralLine l1, IntegralLine l2) {
        float sum = .0f;
        for (int i = 0; i < 3; i++)
            sum += glm::distance(l1.getPositions()[i], l2.getPositions()[i]);
        return sum;
    };

    if (set1.size() != set2.size()) LogWarn("Comparing sets with different number of lines");

    std::vector<vec4> colormap;
    float colorstep = 1.f / set2.size();
    for (int i = 0; i < set2.size(); i++) {
        colormap.push_back(vec4(color::hsv2rgb(vec3(colorstep * i, 1.f, 1.f)), 1.f));
    }

    // Find pairs

    std::vector<LinePair> pairs;
    size_t numNoMatch = 0, numAlreadyMatched = 0, numPairs = 0;
    std::vector<size_t> taken;

    for (size_t li = 0; li < set1.size(); li++) {
        auto l = set1.getVector()[li];
        float minDist = std::numeric_limits<float>::infinity();
        size_t matchedIdx = 0;
        for (size_t i = 0; i < set2.size(); i++) {
            float d = distance(l, set2[i]);
            if (d < minDist) {
                minDist = d;
                matchedIdx = i;
            }
        }
        if (std::find(taken.begin(), taken.end(), matchedIdx) < taken.end()) {
            if (noTriples_) {
                numAlreadyMatched++;
                continue;
            }
        }
        if (minDist > matchTolerance_) {
            numNoMatch++;
            continue;
        }
        taken.push_back(matchedIdx);

        numPairs++;

        auto l_match = set2[matchedIdx];

        util::error(l, l_match);

        if (tubes_) {
            LinePair pair(l, l_match);
            pairs.push_back(pair);
        } else {
            resultSet->push_back(l, matchedIdx);
            resultSet->push_back(l_match, matchedIdx);
            colors->push_back(colormap[matchedIdx]);
            colors->push_back(colormap[matchedIdx]);
        }
    }

    LogInfo("Pairwise comparison: " << numNoMatch << " lines have no match, " << numAlreadyMatched
                                    << " lines found a match that was already taken, " << numPairs
                                    << " pairs found.");

    // Compute mean lines and property ranges

    std::vector<MeanLine> means;
    float minDeviation = std::numeric_limits<float>::infinity(), maxDeviation = 0.0f;
    float minRunlength = std::numeric_limits<float>::infinity(), maxRunlength = 0.0f;
    float minWallDistance = std::numeric_limits<float>::infinity(), maxWallDistance = 0.0f;

    for (LinePair& pair : pairs) {
        MeanLine mean = pair.meanLineUntil(splitThreshold_.get());

        for (const auto d : mean.deviations) {
            if (d < minDeviation) minDeviation = d;
            if (d > maxDeviation) maxDeviation = d;
        }

        auto l = mean.line.getLength();
        if (l < minRunlength) minRunlength = l;
        if (l > maxRunlength) maxRunlength = l;

        // TODO get wall distances from sampler

        means.push_back(mean);
    }

    severenessFilter_.setMaxValue(numPairs);

    std::sort(means.begin(), means.end(),
              [&](MeanLine a, MeanLine b) { return severeness(a) > severeness(b); });

    // Create mesh to render

    for (size_t i = 0; i < severenessFilter_.get(); i++) {
        MeanLine mean = means[i];

        if (!mean.deviations.empty()) {
            tubeGfx(
                mesh, mean.line.getPositions(), [&](int i) { return mean.deviations[i]; },
                [&](int i) {
                    return colorMapping({mean.deviations[i], minDeviation, maxDeviation},
                                        {(float)mean.line.getLength(), minRunlength, maxRunlength});
                });
        }

        if (mean.isPartial) {
            LinePair part = mean.pair->lastPart(mean.partialIndex);
            if (part.l1.getPositions().size() > 0)
                tubeGfx(
                    mesh, part.l1.getPositions(), [&](int i) { return divergedLines_.get(); },
                    [](int i) { return vec4(1); });
            if (part.l2.getPositions().size() > 0)
                tubeGfx(
                    mesh, part.l2.getPositions(), [&](int i) { return divergedLines_.get(); },
                    [](int i) { return vec4(1); });
        }
    }

    out_.setData(resultSet);
    colors_.setData(colors);
    tubeMesh_.setData(mesh);
}

float IntegralLineCompare::severeness(MeanLine mean) {

    float sum = 0.f;
    for (auto d : mean.deviations) {
        sum += d;
    }

    switch (severenessMetric_.get()) {
        case SeverenessMetric::Runlength:
            return 1.f / mean.line.getLength();
        case SeverenessMetric::ThicknessSum:
            return sum;
        default:
            return sum;
    }
}

vec4 IntegralLineCompare::colorMapping(BoundedFloat thickness, BoundedFloat runlength,
                                       BoundedFloat wallDistance) {
    vec4 rgba =
        thickness_.get().sample((thickness.v - thickness.min) / (thickness.max - thickness.min));
    vec3 rgb = vec3(rgba);
    float a = rgba.a;
    if (enableRunlength_) {
        rgba = runlength_.get().sample((runlength.v - runlength.min) /
                                       (runlength.max - runlength.min));
        rgb += vec3(rgba) * rgba.a;
        a += rgba.a;
    }
    if (enableWallDistance_) {
        rgba = wallDistance_.get().sample((wallDistance.v - wallDistance.min) /
                                          (wallDistance.max - wallDistance.min));
        rgb += vec3(rgba) * rgba.a;
        a += rgba.a;
    }
    return vec4(rgb, a);
}

void IntegralLineCompare::tubeGfx(std::shared_ptr<MyLineMesh> mesh, std::vector<dvec3> vertices,
                                  std::function<float(int)> radius,
                                  std::function<vec4(int)> color) {
    auto meshIndices = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::StripAdjacency);

    //FIXME first and last line segment normals

    for (size_t i = 0; i < vertices.size(); i++) {
        if (i == 0) {
            vec3 dir = glm::normalize(vertices[i + 1] - vertices[i]);
            meshIndices->add(mesh->addVertex(vertices[0], radius(0), dir, color(0)));
            meshIndices->add(mesh->addVertex(vertices[0], radius(0), dir, color(0)));
        } else if (i > 0 && i < (vertices.size() - 1)) {
            vec3 dir = glm::normalize(vertices[i + 1] - vertices[i]);
            vec3 prevDir = glm::normalize(vertices[i] - vertices[i - 1]);
            vec3 normal = glm::normalize(dir + prevDir);
            meshIndices->add(mesh->addVertex(vertices[i], radius(i), normal, color(i)));
        } else {
            vec3 prevDir = glm::normalize(vertices[i] - vertices[i - 1]);
            meshIndices->add(mesh->addVertex(vertices.back(), radius(i), prevDir, color(i)));
            meshIndices->add(mesh->addVertex(vertices.back(), radius(i), prevDir, color(i)));
        }
    }
}

}  // namespace inviwo
