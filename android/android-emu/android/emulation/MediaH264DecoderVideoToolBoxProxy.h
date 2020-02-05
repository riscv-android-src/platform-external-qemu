// Copyright (C) 2020 The Android Open Source Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "android/emulation/GoldfishMediaDefs.h"
#include "android/emulation/H264NaluParser.h"
#include "android/emulation/MediaCodec.h"
#include "android/emulation/MediaH264DecoderFfmpeg.h"
#include "android/emulation/MediaH264DecoderVideoToolBox.h"

#include <VideoToolbox/VideoToolbox.h>

#include <cstdint>
#include <string>
#include <vector>

#include <stdio.h>
#include <string.h>

#include <stddef.h>

namespace android {
namespace emulation {

class MediaH264DecoderVideoToolBoxProxy : public MediaH264DecoderPlugin {
public:


    virtual void initH264Context(unsigned int width,
                                 unsigned int height,
                                 unsigned int outWidth,
                                 unsigned int outHeight,
                                 PixelFormat pixFmt) override;
    virtual void reset(unsigned int width,
                                 unsigned int height,
                                 unsigned int outWidth,
                                 unsigned int outHeight,
                                 PixelFormat pixFmt) override;
    virtual MediaH264DecoderPlugin* clone() override;
    virtual void destroyH264Context() override;
    virtual void decodeFrame(void* ptr, const uint8_t* frame, size_t szBytes, uint64_t pts) override;
    virtual void flush(void* ptr) override;
    virtual void getImage(void* ptr) override;

    explicit MediaH264DecoderVideoToolBoxProxy(uint32_t version);
    virtual ~MediaH264DecoderVideoToolBoxProxy();

private:
    using DecoderState = MediaH264DecoderVideoToolBox::DecoderState;

    uint32_t mVersion = 100;
    unsigned int mWidth;
    unsigned int mHeight;
    unsigned int mOutputWidth;
    unsigned int mOutputHeight;
    PixelFormat  mOutputPixelFormat;

    std::vector<uint8_t> mSPS;
    std::vector<uint8_t> mPPS;
    bool mIsVideoToolBoxDecoderInGoodState = true;
    MediaH264DecoderVideoToolBox mVideoToolBoxDecoder;
    MediaH264DecoderFfmpeg mFfmpegDecoder;
    MediaH264DecoderPlugin* mCurrentDecoder = nullptr;
    std::vector<uint8_t> prefixNaluHeader(std::vector<uint8_t> data);

}; // MediaH264DecoderVideoToolBoxProxy


}  // namespace emulation
}  // namespace android
