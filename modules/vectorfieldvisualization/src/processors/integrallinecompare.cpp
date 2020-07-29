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
    , tubes_("tubes", "Tubes") {

    addPort(lines1_);
    addPort(lines2_);
    addPort(out_);
    addPort(tubeMesh_);
    addPort(colors_);

    addProperties(matchTolerance_, noTriples_, tubes_);
}

void IntegralLineCompare::process() {
    const auto set1 = *lines1_.getData();
    const auto set2 = *lines2_.getData();
    const auto resultSet = std::make_shared<IntegralLineSet>(mat4(1));
    const auto colors = std::make_shared<std::vector<vec4>>();

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
            if (noTriples_) continue;
        }
        if (minDist > matchTolerance_) continue;
        taken.push_back(matchedIdx);

        auto l_match = set2[matchedIdx];

        util::error(l, l_match);

        if (tubes_) {
            resultSet->push_back(LinePair(l, l_match).meanLine(), matchedIdx);
        } else {
            resultSet->push_back(l, matchedIdx);
            resultSet->push_back(l_match, matchedIdx);
            colors->push_back(colormap[matchedIdx]);
            colors->push_back(colormap[matchedIdx]);
        }
    }

    out_.setData(resultSet);
    colors_.setData(colors);
}

}  // namespace inviwo
