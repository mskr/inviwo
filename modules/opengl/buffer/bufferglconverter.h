/**********************************************************************
 * Copyright (C) 2013 Scientific Visualization Group - Link�ping University
 * All Rights Reserved.
 * 
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * No part of this software may be reproduced or transmitted in any
 * form or by any means including photocopying or recording without
 * written permission of the copyright owner.
 *
 * Primary author : Daniel J�nsson
 *
 **********************************************************************/

#ifndef IVW_BUFFERGL_CONVERTER_H
#define IVW_BUFFERGL_CONVERTER_H

#include <modules/opengl/openglmoduledefine.h>
#include <modules/opengl/inviwoopengl.h>
#include <modules/opengl/geometry/attributebuffergl.h>
#include <inviwo/core/datastructures/buffer/bufferram.h>
#include <inviwo/core/datastructures/geometry/attributes.h>


namespace inviwo {

class IVW_MODULE_OPENGL_API BufferRAM2GLConverter : public RepresentationConverterType<BufferGL> {

public:
    BufferRAM2GLConverter();
    virtual ~BufferRAM2GLConverter();

    inline bool canConvertFrom(const DataRepresentation* source) const {
        return dynamic_cast<const BufferRAM*>(source) != NULL;
    }
    DataRepresentation* createFrom(const DataRepresentation* source);
    void update(const DataRepresentation* source, DataRepresentation* destination);
};

class IVW_MODULE_OPENGL_API BufferGL2RAMConverter : public RepresentationConverterType<BufferRAM> {

public:
    BufferGL2RAMConverter();
    virtual ~BufferGL2RAMConverter();

    inline bool canConvertFrom(const DataRepresentation* source) const {
        return dynamic_cast<const BufferGL*>(source) != NULL;
    }

    DataRepresentation* createFrom(const DataRepresentation* source);
    void update(const DataRepresentation* source, DataRepresentation* destination);
};


} // namespace

#endif // IVW_BUFFERGL_CONVERTER_H
