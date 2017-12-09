/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2017 Inviwo Foundation
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

#ifndef IVW_GLUISLIDER_H
#define IVW_GLUISLIDER_H

#include <modules/userinterfacegl/userinterfaceglmoduledefine.h>
#include <inviwo/core/common/inviwo.h>

#include <modules/userinterfacegl/glui/element.h>

namespace inviwo {

class Texture2DArray;

namespace glui {

class Renderer;

/**
 * \class Button
 * \brief glui::element representing a button with the label centered within
 */
class IVW_MODULE_USERINTERFACEGL_API Slider : public Element {
public:
    Slider(const std::string &label, int value, int minValue, int maxValue, Processor &processor,
           Renderer &uiRenderer, const ivec2 &extent = ivec2(100, 24));
    virtual ~Slider() = default;

    void set(int value);
    void set(int value, int minValue, int maxValue);
    int get() const;
    int getMinValue() const;
    int getMaxValue() const;

protected:
    virtual void renderWidget(const ivec2 &origin) override;

    int getPreviousValue() const;

private:
    virtual ivec2 computeLabelPos(int descent) const override;
    virtual UIState uiState() const override;
    virtual vec2 marginScale() const override;
    virtual void pushStateChanged() override;

    Texture2DArray *uiTextures_;
    Texture2DArray *grooveTextures_;

    int value_;
    int min_;
    int max_;

    int prevValue_;
};

}  // namespace glui

}  // namespace inviwo

#endif  // IVW_GLUISLIDER_H