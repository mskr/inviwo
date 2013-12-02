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
 * Primary author : Rickard Englund
 *
 **********************************************************************/

#ifndef IVW_PYCAMERAMEHTODINVIWO_H
#define IVW_PYCAMERAMEHTODINVIWO_H


#include <modules/python/pythonmoduledefine.h>

#include "../pythoninterface/pymethod.h"

namespace inviwo {
PyObject* py_setCameraFocus(PyObject* /*self*/, PyObject* /*args*/);
PyObject* py_setCameraUp(PyObject* /*self*/, PyObject* /*args*/);
PyObject* py_setCameraPos(PyObject* /*self*/, PyObject* /*args*/);


    class IVW_MODULE_PYTHON_API PySetCameraFocusMethod : public PyMethod{
    public:
        char *getName(){return "setCameraFocus";}
        char *getDesc(){return "setCameraFocus(processor name, property id, tuple)\tFunction to set the cameras focal point.";}
        virtual PyCFunction getFunc(){return py_setCameraFocus;}

    };

    class IVW_MODULE_PYTHON_API PySetCameraUpMethod : public PyMethod{
    public:
        char *getName(){return "setCameraUp";}
        char *getDesc(){return "setCameraUpDirection(processor name, property id, tuple)\tFunction to set the cameras up direction.";}
        virtual PyCFunction getFunc(){return py_setCameraUp;}

    };

    class IVW_MODULE_PYTHON_API PySetCameraPosMethod : public PyMethod{
    public:
        char *getName(){return "setCameraPosition";}
        char *getDesc(){return "setCameraUpDirection(processor name, property id, tuple)\tFunction to set the cameras position.";}
        virtual PyCFunction getFunc(){return py_setCameraPos;}

    };

} //namespace





#endif // IVW_PYCAMERAMEHTODINVIWO_H


