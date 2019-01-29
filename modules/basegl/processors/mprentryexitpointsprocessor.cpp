/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2012-2018 Inviwo Foundation
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

#include <modules/basegl/processors/mprentryexitpointsprocessor.h>
#include <inviwo/core/io/serialization/versionconverter.h>
#include <modules/opengl/buffer/buffergl.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/opengl/shader/shaderutils.h>

namespace inviwo {

const ProcessorInfo MPREntryExitPoints::processorInfo_{
    "org.inviwo.MPREntryExitPoints",  // Class identifier
    "MPR Entry Exit Points",          // Display name
    "Mesh Rendering",                 // Category
    CodeState::Experimental,          // Code state
    Tags::GL,                         // Tags
};
const ProcessorInfo MPREntryExitPoints::getProcessorInfo() const { return processorInfo_; }

MPREntryExitPoints::MPREntryExitPoints()
    : Processor()
    , volumeInport_("volume")
    , entryPort_("entry", DataVec4UInt16::get())
    , exitPort_("exit", DataVec4UInt16::get())
    , offset0_("offset0", "Offset 0", -0.01f, -1.0f, 1.0f, 0.001f)
    , offset1_("offset1", "Offset 1", 0.01f, -1.0f, 1.0f, 0.001f)
    , zoomFactor_("zoomFactor", "Zoom Factor", 1.0f, 0.01f, 100.0f, 0.01f)
    , correctionAngle_("correctionAngle", "Correction Angle", 0.0f, -1e9f, 1e9f, 0.001f)
    , canvasSize_("canvasSize", "Canvas Size", ivec2(0), ivec2(0), ivec2(8096), ivec2(1))
    , volumeDimensions_("volumeDimensions", "Volume Dimensions", size3_t(0), size3_t(0), size3_t(std::numeric_limits<size_t>::max()), size3_t(1))
    , volumeSpacing_("volumeSpacing", "Volume Spacing", vec3(0.0f), vec3(0.0f), vec3(1e5f), vec3(1e-3f))
    , cursorScreenPos_("cursorScreenPos", "Cursor Screen Pos", vec2(0.5f), vec2(0.0f), vec2(1.0f))
    , cursorScreenPosOld_("cursorScreenPosOld", "Cursor Screen Pos Old", cursorScreenPos_.get(), vec2(0.0f), vec2(1.0f))
    , shader_("uv_pass_through.vert", "mpr_entry_exit_points.frag")
    , mprP_("mprP_", "mprP_", vec3(0.5f), vec3(-10.0f), vec3(10.0f))
    , mprBasisR_("mprBasisR_", "mprBasisR_", vec3(0.0f), vec3(-1.0f), vec3(1.0f))
    , mprBasisU_("mprBasisU_", "mprBasisU_", vec3(0.0f), vec3(-1.0f), vec3(1.0f))
    , mprBasisN_("mprBasisN_", "mprBasisN_", vec3(0.0f), vec3(-1.0f), vec3(1.0f))
{
    addPort(volumeInport_);
    addPort(entryPort_, "ImagePortGroup1");
    addPort(exitPort_, "ImagePortGroup1");

    addProperty(offset0_);
    addProperty(offset1_);
    addProperty(zoomFactor_);
    addProperty(correctionAngle_);
    addProperty(canvasSize_);
    addProperty(volumeDimensions_);
    addProperty(volumeSpacing_);

    cursorScreenPos_.onChange([this]() {
        const auto cursor_offset = cursorScreenPos_.get() - cursorScreenPosOld_.get();
        cursorScreenPosOld_ = cursorScreenPos_;
    });
    addProperty(cursorScreenPos_);
    cursorScreenPosOld_.setReadOnly(true);
    addProperty(cursorScreenPosOld_);

    shader_.onReload([this]() { invalidate(InvalidationLevel::InvalidResources); });

    addProperty(mprP_);
    addProperty(mprBasisR_);
    addProperty(mprBasisU_);
    addProperty(mprBasisN_);
}

MPREntryExitPoints::~MPREntryExitPoints() {}

void MPREntryExitPoints::process() {
    auto const quad = util::makeBuffer<vec2>({
        { -1.0f, -1.0f },{ 1.0f, -1.0f },{ -1.0f, 1.0f },{ 1.0f, 1.0f } 
    });

    // generate entry points
    utilgl::activateAndClearTarget(*entryPort_.getEditableData().get(), ImageType::ColorOnly);
    shader_.activate();

    shader_.setUniform("p_screen", cursorScreenPos_.get()); // plane pos. in screen space
    shader_.setUniform("p", mprP_.get()); // volume position
    shader_.setUniform("n", mprBasisN_.get()); // plane's normal
    shader_.setUniform("u", mprBasisU_.get()); // plane's up
    shader_.setUniform("r", mprBasisR_.get()); // plane's right
    shader_.setUniform("thickness_offset", offset0_.get()); // plane's offset along normal, // note that setUniform does not work when passing a literal 0
    shader_.setUniform("zoom_factor", zoomFactor_.get()); // zoom factor
    shader_.setUniform("correction_angle", -correctionAngle_.get()); // correction angle
    shader_.setUniform("canvas_size", vec2(canvasSize_.get())); // correction angle
    shader_.setUniform("volume_dimensions", vec3(volumeDimensions_.get())); // correction angle
    shader_.setUniform("volume_spacing", volumeSpacing_.get()); // correction angle

    /*LogInfo("cursorScreenPos_: " << cursorScreenPos_);
    LogInfo("offset0_: " << offset0_);
    LogInfo("offset1_: " << offset1_);
    LogInfo("mprP_: " << mprP_);
    LogInfo("mprBasisR_: " << mprBasisR_);
    LogInfo("mprBasisU_: " << mprBasisU_);
    LogInfo("mprBasisN_: " << mprBasisN_);
    LogInfo("canvasSize_: " << canvasSize_);
    LogInfo("volumeDimensions_: " << volumeDimensions_);
    LogInfo("volumeSpacing_: " << volumeSpacing_);*/

    auto quadGL = quad->getRepresentation<BufferGL>();
    quadGL->enable();
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    // generate exit points
    utilgl::activateAndClearTarget(*exitPort_.getEditableData().get(), ImageType::ColorOnly);
    shader_.setUniform("thickness_offset", offset1_.get());
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    quadGL->disable();
}

void MPREntryExitPoints::deserialize(Deserializer& d) {
    util::renamePort(d, {{&entryPort_, "entry-points"}, {&exitPort_, "exit-points"}});
    Processor::deserialize(d);
}

}  // namespace