/*
* Copyright (C) 2011 The Android Open Source Project
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
#include "WindowSurface.h"

#include "FbConfig.h"
#include "EGLDispatch.h"
#include "GLErrorLog.h"

#include <GLES/glext.h>

#include <stdio.h>
#include <string.h>


WindowSurface::WindowSurface(EGLDisplay display,
                             EGLConfig config) :
        mSurface(NULL),
        mAttachedColorBuffer(NULL),
        mReadContext(NULL),
        mDrawContext(NULL),
        mWidth(0),
        mHeight(0),
        mConfig(config),
        mDisplay(display) {}

WindowSurface::~WindowSurface() {
    s_egl.eglDestroySurface(mDisplay, mSurface);
}

WindowSurface *WindowSurface::create(EGLDisplay display,
                                     EGLConfig config,
                                     int p_width,
                                     int p_height) {
    // allocate space for the WindowSurface object
    WindowSurface *win = new WindowSurface(display, config);
    if (!win) {
        return NULL;
    }

    // Create a pbuffer to be used as the egl surface
    // for that window.
    if (!win->resize(p_width, p_height)) {
        delete win;
        return NULL;
    }

    return win;
}


// flushColorBuffer - The function makes sure that the
//    previous attached color buffer is updated, if copy or blit should be done
//    in order to update it - it is being done here.
bool WindowSurface::flushColorBuffer() {
    if (mAttachedColorBuffer.Ptr() != NULL) {
        return blitToColorBuffer();
    } else {
        return true;
    }
}

// setColorBuffer - this function is called when a new color buffer needs to
//    be attached to the surface. The function doesn't make sure that the
//    previous attached color buffer is updated, this is done by flushColorBuffer
void WindowSurface::setColorBuffer(ColorBufferPtr p_colorBuffer) {
    mAttachedColorBuffer = p_colorBuffer;

    // resize the window if the attached color buffer is of different
    // size.
    unsigned int cbWidth = mAttachedColorBuffer->getWidth();
    unsigned int cbHeight = mAttachedColorBuffer->getHeight();

    if (cbWidth != mWidth || cbHeight != mHeight) {
        resize(cbWidth, cbHeight);
    }
}

//
// This function is called after the context and eglSurface is already
// bound in the current thread (eglMakeCurrent has been called).
// This function should take actions required on the other surface objects
// when being bind/unbound
//
void WindowSurface::bind(RenderContextPtr p_ctx, BindType p_bindType) {
    if (p_bindType == BIND_READ) {
        mReadContext = p_ctx;
    } else if (p_bindType == BIND_DRAW) {
        mDrawContext = p_ctx;
    } else if (p_bindType == BIND_READDRAW) {
        mReadContext = p_ctx;
        mDrawContext = p_ctx;
    }
}

bool WindowSurface::blitToColorBuffer() {
    if (!mWidth && !mHeight) {
        return false;
    }

    if (mAttachedColorBuffer->getWidth() != mWidth ||
        mAttachedColorBuffer->getHeight() != mHeight) {
        // XXX: should never happen - how this needs to be handled?
        fprintf(stderr, "Dimensions do not match\n");
        return false;
    }

    if (!mDrawContext.Ptr()) {
        fprintf(stderr, "Draw context is NULL\n");
        return false;
    }

    // Make the surface current
    EGLContext prevContext = s_egl.eglGetCurrentContext();
    EGLSurface prevReadSurf = s_egl.eglGetCurrentSurface(EGL_READ);
    EGLSurface prevDrawSurf = s_egl.eglGetCurrentSurface(EGL_DRAW);

    if (!s_egl.eglMakeCurrent(mDisplay,
                              mSurface,
                              mSurface,
                              mDrawContext->getEGLContext())) {
        fprintf(stderr, "Error making draw context current\n");
        return false;
    }

    mAttachedColorBuffer->blitFromCurrentReadBuffer();

    // restore current context/surface
    s_egl.eglMakeCurrent(mDisplay, prevDrawSurf, prevReadSurf, prevContext);

    return true;
}

bool WindowSurface::resize(unsigned int p_width, unsigned int p_height)
{
    if (mSurface && mWidth == p_width && mHeight == p_height) {
        // no need to resize
        return true;
    }

    EGLContext prevContext = s_egl.eglGetCurrentContext();
    EGLSurface prevReadSurf = s_egl.eglGetCurrentSurface(EGL_READ);
    EGLSurface prevDrawSurf = s_egl.eglGetCurrentSurface(EGL_DRAW);
    EGLSurface prevPbuf = mSurface;
    bool needRebindContext = mSurface &&
                             (prevReadSurf == mSurface ||
                              prevDrawSurf == mSurface);

    if (needRebindContext) {
        s_egl.eglMakeCurrent(
                mDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    }

    //
    // Destroy previous surface
    //
    if (mSurface) {
        s_egl.eglDestroySurface(mDisplay, mSurface);
        mSurface = NULL;
    }

    //
    // Create pbuffer surface.
    //
    EGLint pbufAttribs[5];
    pbufAttribs[0] = EGL_WIDTH;
    pbufAttribs[1] = p_width;
    pbufAttribs[2] = EGL_HEIGHT;
    pbufAttribs[3] = p_height;
    pbufAttribs[4] = EGL_NONE;

    mSurface = s_egl.eglCreatePbufferSurface(mDisplay,
                                             mConfig,
                                             pbufAttribs);
    if (mSurface == EGL_NO_SURFACE) {
        fprintf(stderr, "Renderer error: failed to create/resize pbuffer!!\n");
        return false;
    }

    mWidth = p_width;
    mHeight = p_height;

    if (needRebindContext) {
        s_egl.eglMakeCurrent(
                mDisplay,
                (prevDrawSurf == prevPbuf) ? mSurface : prevDrawSurf,
                (prevReadSurf == prevPbuf) ? mSurface : prevReadSurf,
                prevContext);
    }

    return true;
}
