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

namespace inviwo {

const ProcessorInfo IntegralLineCompare::getProcessorInfo() const { return processorInfo_; }

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo IntegralLineCompare::processorInfo_{
    "org.inviwo.IntegralLineCompare",  // Class identifier
    "Integral Line Compare",         // Display name
    "Vector Field Visualization",           // Category
    CodeState::Stable,                      // Code state
    Tags::CPU,                              // Tags
};

IntegralLineCompare::IntegralLineCompare()
    : Processor(), lines1_("lines1"), lines2_("lines2"), out_("out"), colors_("colors") {

    addPort(lines1_);
    addPort(lines2_);
    addPort(out_);
    addPort(colors_);
}

void IntegralLineCompare::process() {
    const auto a = *lines1_.getData();
    const auto b = *lines2_.getData();
    const auto out = std::make_shared<IntegralLineSet>(mat4(1));
    const auto colors = std::make_shared<std::vector<vec4>>();
    for (size_t i = 0; i < a.size(); i++) {
        const auto line = a.at(i);
        const auto seed = line.getPositions().back();
        out->push_back(line, i);
        colors->push_back(vec4(.5f, .5f, 0.f, 1.f));
    }
    for (size_t i = 0; i < b.size(); i++) {
        out->push_back(b.at(i), a.size() + i);
        colors->push_back(vec4(0.f, .5f, .5f, 1.f));
    }
    out_.setData(out);
    colors_.setData(colors);
}

}  // namespace inviwo
