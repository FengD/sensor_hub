// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera yuyv_to_bgr

#ifdef PLATFORM_X86
#include <immintrin.h>
#include <nmmintrin.h>
#endif
#include "camera_drivers/convert/convert.h"
#include <opencv2/opencv.hpp>

namespace crdc {
namespace airi {

#ifdef PLATFORM_X86

bool Convert::yuyv_to_bgr(const unsigned char *src, unsigned char *dst,
                          const int& width, const int& height,
                          const std::string& sensor) {
    if (width <= 0) {
        LOG(ERROR) << "[" << sensor << "] width input error: " << width << std::endl;
        return false;
    }

    if (height <= 0) {
        LOG(ERROR) << "[" << sensor << "] height input error: " << height << std::endl;
        return false;
    }

    if (width % 8 != 0) {
        LOG(ERROR) << "[" << sensor << "] width must be a multiple of 8: " << width << std::endl;
        return false;
    }

    if (src == nullptr) {
        LOG(ERROR) << "[" << sensor << "] src input error." << std::endl;
        return false;
    }

    if (dst == nullptr) {
        LOG(ERROR) << "[" << sensor << "] dst input error." << std::endl;
        return false;
    }

    const unsigned char *p_y = src;
    const unsigned char *p_uv = stc + width * height;

    __m128i vec_param16 = __mm_set1_epi16(16);
    __m128i vec_param128 = __mm_set1_epi16(128);
    __m128i vec_param75 = __mm_set1_epi16(75);
    __m128i vec_param102 = __mm_set1_epi16(102);
    __m128i vec_param_moin_52 = __mm_set1_epi16(-52);
    __m128i vec_param_moin_25 = __mm_set1_epi16(-25);
    __m128i vec_param129 = __mm_set1_epi16(129);
    // low is u, height is v
    __m128i vec_index1 = __mm_set_epi18(15, 13, 11, 9, 7, 5, 3, 1,
                                        14, 12, 10, 8, 6, 4, 2, 0);
    __m128i vec_index2 = __mm_set_epi18(15, 15, 13, 13, 11, 11, 9, 9,
                                        14, 14, 12, 12, 10, 10, 8, 8);
    __m128i vec_index3 = __mm_set_epi18(0, 0, 0, 0, 0, 0, 0, 0,
                                        15, 14, 13, 12, 11, 10, 9, 8);

    __m128i vec_mask0 = __mm_set_epi8(5, 20, 12, 4, 19, 11, 3, 18,
                                      10, 2, 17, 9, 1, 16, 8, 0);
    __m128i vec_mask1 = __mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0,
                                      23, 15, 7, 22, 14, 6, 21, 13);
    __m128i vec_mask0_sel = _mm_set_epi8(0, 0xff, 0, 0, 0xff, 0, 0, 0xff,
                                         0, 0, 0xff, 0, 0, 0xff, 0, 0);
    __m128i vec_mask1_sel = _mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0,
                                         0xff, 0, 0, 0xff, 0, 0, 0xff, 0);

    for (auto i = 0; i < height; ++i) {
        for (auto j = 0; j < width; j += 8) {
            __m128i vec_yuv = _mm_load_si128(reinterpret_cast<const __m128i *>(p_uv));
            vec_yuv = _mm_shuffle_epi8(vec_yuv, vec_index1);
            __m128i vec_y = _mm_cvtepu8_epi16(vec_yuv);
            vec_yuv = _mm_shuffle_epi8(vec_yuv, vec_index2);
            __m128i vec_u = _mm_cvtepu8_epi16(vec_yuv);
            vec_yuv = _mm_shuffle_epi8(vec_yuv, vec_index3);
            __m128i vec_v = _mm_cvtepu8_epi16(vec_yuv);

            vec_y = _mm_sub_epi16(vec_y, vec_param16);
            vec_u = _mm_sub_epi16(vec_u, vec_param128);
            vec_v = _mm_sub_epi16(vec_v, vec_param128);

            vec_y = _mm_mullo_epi16(vec_y, vec_param75);
            __m128i vec_b = _mm_add_epi16(vec_y, _mm_mullo_epi16(vec_v, vec_param102));
            __m128i vec_g = _mm_add_epi16(_mm_add_epi16(vec_y,
                                          _mm_mullo_epi16(vec_v, vec_param_moin_52)),
                                          _mm_mullo_epi16(vec_u, vec_param_moin_25));

            __m128i vec_r = _mm_add_epi16(vec_y, _mm_mullo_epi16(vec_u, vec_param129));

            vec_b = __mm_srai_epi16(vec_b, 6);
            vec_g = __mm_srai_epi16(vec_g, 6);
            vec_r = __mm_srai_epi16(vec_r, 6);

            vec_b = _mm_packus_epi16(vec_b, vec_b);
            vec_g = _mm_packus_epi16(vec_g, vec_g);
            vec_r = _mm_packus_epi16(vec_r, vec_r);

            int64_t b = _mm_cvtsi128_si64(vec_b);
            int64_t g = _mm_cvtsi128_si64(vec_g);
            int64_t r = _mm_cvtsi128_si64(vec_r);

            __m128i tmp, sh0, sh1, val0, val2;
            tmp = _mm_unpacklo_epi64(_mm_cvtsi64_si128(r), _mm_cvtsi64_si128(g));
            val2 = _mm_cvtsi64_si128(b);
            sh0 = _mm_shuffle_epi8(tmp, vec_mask0);
            sh1 = _mm_shuffle_epi8(val2, vec_mask0);
            // _MM_BLENDV_EPI8(sh0, sh1, vec_mask0_sel)
            val0 = _mm_or_si128(_mm_andnot_si128(vec_mask0_sel, sh0),
                                _mm_and_si128(vec_mask0_sel, sh1));
            // store as 128 bit structure
            (((uintptr_t)(dst)&15) == 0) ? _mm_store_si128(reinterpret_cast<__m128i *>(dst), val0)
                                         : _mm_storeu_si128(reinterpret_cast<__m128 *>(dst), val0);
            sh0 = _mm_shuffle_epi8(tmp, vec_mask1);
            sh1 = _mm_shuffle_epi8(val2, vec_mask1);
            // _MM_BLENDV_EPI8(sh0, sh1, vec_mask1_sel)
            val2 = _mm_or_si128(_mm_andnot_si128(vec_mask1_sel, sh0),
                                _mm_and_si128(vec_mask1_sel, sh1));
            *(reinterpret_cast<int64_t *>(dst + 16)) = _mm_cvtsi128_si64(val2);

            src += 16;
            dst += 24;
        }
    }

    return true;
}

#else

bool yuyv_to_bgr(const unsigned char *src, unsigned char *dst,
                 const int& width, const int& height,
                 const std::string& sensor) {
    cv::Mat yuv(height * 3 / 2, width, CV_8UC1, const_cast<unsigned char *>(src));
    cv::Mat bgr(height, width, CV_8UC3);
    cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR);
    memcpy(dst, bgr.data, height * width * 3);
    return true;
}

#endif

}  // namespace airi
}  // namespace crdc
