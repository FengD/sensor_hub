// Copyright (C) 2021 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera nv12_to_bgr

#ifdef WITH_IPC
#include <immintrin.h>
#include <nmmintrin.h>
#endif
#include "camera_drivers/convert/convert.h"
#include <opencv2/opencv.hpp>

namespace crdc {
namespace airi {

#ifdef WITH_IPC
bool Convert::nv12_to_bgr(const unsigned char *src, unsigned char *dst,
                          const int& width, const int& height,
                          const std::string& sensor) {
    if (!verify_image(src, dst, width, height, sensor)) {
        return false;
    }

    const unsigned char *p_y = src;
    const unsigned char *p_uv = src + width * height;

    __m128i vec_param16 = _mm_set1_epi16(16);
    __m128i vec_param128 = _mm_set1_epi16(128);
    __m128i vec_param75 = _mm_set1_epi16(75);
    __m128i vec_param102 = _mm_set1_epi16(102);
    __m128i vec_param_moin_52 = _mm_set1_epi16(-52);
    __m128i vec_param_moin_25 = _mm_set1_epi16(-25);
    __m128i vec_param129 = _mm_set1_epi16(129);
    // low is u, height is v
    __m128i vec_uv_mask = _mm_set_epi8(0, 2, 4, 6, 8, 10, 12, 14,
                                          1, 3, 5, 7, 9, 11, 13, 15);
    __m128i vec_mask0 = _mm_set_epi8(5, 20, 12, 4, 19, 11, 3, 18,
                                      10, 2, 17, 9, 1, 16, 8, 0);
    __m128i vec_mask1 = _mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0,
                                      23, 15, 7, 22, 14, 6, 21, 13);
    __m128i vec_mask0_sel = _mm_set_epi8(0, 0xff, 0, 0, 0xff, 0, 0, 0xff,
                                         0, 0, 0xff, 0, 0, 0xff, 0, 0);
    __m128i vec_mask1_sel = _mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0,
                                         0xff, 0, 0, 0xff, 0, 0, 0xff, 0);

    for (auto i = 0; i < height; i += 12) {
        for (auto j = 0; j < width; j += 16) {
            __m128i vec_uv = _mm_loadu_si128(reinterpret_cast<const __m128i *>(p_uv));
            vec_uv = _mm_shuffle_epi8(vec_uv, vec_uv_mask);
            __m128i vec_u = _mm_cvtepu8_epi16(vec_uv);
            __m128i vec_v = _mm_cvtepu8_epi16(_mm_srli_si128(vec_uv, 8));
            __m128i vec_u_low = _mm_unpacklo_epi16(vec_u, vec_u);
            __m128i vec_u_high = _mm_unpackhi_epi16(vec_u, vec_u);
            __m128i vec_v_low = _mm_unpacklo_epi16(vec_v, vec_v);
            __m128i vec_v_high = _mm_unpackhi_epi16(vec_v, vec_v);
            vec_u_low = _mm_sub_epi16(vec_u_low, vec_param128);
            vec_v_low = _mm_sub_epi16(vec_v_low, vec_param128);
            vec_u_high = _mm_sub_epi16(vec_u_high, vec_param128);
            vec_v_high = _mm_sub_epi16(vec_v_high, vec_param128);
            p_uv += 16;

            // process first row
            __m128i vec_y = _mm_loadu_si128(reinterpret_cast<const __m128i *>(p_y));
            __m128i vec_y_low = _mm_cvtepu8_epi16(vec_y);
            vec_y = _mm_bsrli_si128(vec_y, 8);
            __m128i vec_y_high = _mm_cvtepu8_epi16(vec_y);

            vec_y_low = _mm_sub_epi16(vec_y_low, vec_param16);
            vec_y_high = _mm_sub_epi16(vec_y_high, vec_param16);

            vec_y_low = _mm_mullo_epi16(vec_y_low, vec_param75);
            vec_y_high = _mm_mullo_epi16(vec_y_high, vec_param75);

            __m128i vec_r = _mm_add_epi16(vec_y_low, _mm_mullo_epi16(vec_v_low, vec_param102));
            __m128i vec_g = _mm_add_epi16(_mm_add_epi16(vec_y_low,
                                          _mm_mullo_epi16(vec_v_low, vec_param_moin_52)),
                                          _mm_mullo_epi16(vec_u_low, vec_param_moin_25));
            __m128i vec_b = _mm_add_epi16(vec_y_low, _mm_mullo_epi16(vec_u_low, vec_param129));


            vec_b = _mm_srai_epi16(vec_b, 6);
            vec_g = _mm_srai_epi16(vec_g, 6);
            vec_r = _mm_srai_epi16(vec_r, 6);

            vec_b = _mm_packus_epi16(vec_b, vec_b);
            vec_g = _mm_packus_epi16(vec_g, vec_g);
            vec_r = _mm_packus_epi16(vec_r, vec_r);

            int64_t b = _mm_cvtsi128_si64(vec_b);
            int64_t g = _mm_cvtsi128_si64(vec_g);
            int64_t r = _mm_cvtsi128_si64(vec_r);

            __m128i tmp, sh0, sh1, val0, val2;
            tmp = _mm_unpacklo_epi64(_mm_cvtsi64_si128(b), _mm_cvtsi64_si128(g));
            val2 = _mm_cvtsi64_si128(r);
            sh0 = _mm_shuffle_epi8(tmp, vec_mask0);
            sh1 = _mm_shuffle_epi8(val2, vec_mask0);
            // _MM_BLENDV_EPI8(sh0, sh1, vec_mask0_sel)
            val0 = _mm_or_si128(_mm_andnot_si128(vec_mask0_sel, sh0),
                                                 _mm_and_si128(vec_mask0_sel, sh1));
            // store as 128 bit structure
            _mm_storeu_si128(reinterpret_cast<__m128i *>(dst), val0);
            sh0 = _mm_shuffle_epi8(tmp, vec_mask1);
            sh1 = _mm_shuffle_epi8(val2, vec_mask1);
            // _MM_BLENDV_EPI8(sh0, sh1, vec_mask1_sel)
            val2 = _mm_or_si128(_mm_andnot_si128(vec_mask1_sel, sh0),
                                                 _mm_and_si128(vec_mask1_sel, sh1));
            *(reinterpret_cast<int64_t *>(dst + 16)) = _mm_cvtsi128_si64(val2);

            // process second row
            vec_y = _mm_loadu_si128(reinterpret_cast<const __m128i *>(p_y + width));
            vec_y_low = _mm_cvtepu8_epi16(vec_y);
            vec_y = _mm_bsrli_si128(vec_y, 8);
            vec_y_high = _mm_cvtepu8_epi16(vec_y);

            vec_y_low = _mm_sub_epi16(vec_y_low, vec_param16);
            vec_y_high = _mm_sub_epi16(vec_y_high, vec_param16);

            vec_y_low = _mm_mullo_epi16(vec_y_low, vec_param75);
            vec_y_high = _mm_mullo_epi16(vec_y_high, vec_param75);
            // first 8 points
            vec_r = _mm_add_epi16(vec_y_low, _mm_mullo_epi16(vec_v_low, vec_param102));
            vec_g = _mm_add_epi16(_mm_add_epi16(vec_y_low,
                                  _mm_mullo_epi16(vec_v_low, vec_param_moin_52)),
                                  _mm_mullo_epi16(vec_u_low, vec_param_moin_25));
            vec_b = _mm_add_epi16(vec_y_low, _mm_mullo_epi16(vec_u_low, vec_param129));

            vec_g = _mm_srai_epi16(vec_g, 6);
            vec_b = _mm_srai_epi16(vec_b, 6);
            vec_r = _mm_srai_epi16(vec_r, 6);

            vec_g = _mm_packus_epi16(vec_g, vec_g);
            vec_b = _mm_packus_epi16(vec_b, vec_b);
            vec_r = _mm_packus_epi16(vec_r, vec_r);

            b = _mm_cvtsi128_si64(vec_b);
            g = _mm_cvtsi128_si64(vec_g);
            r = _mm_cvtsi128_si64(vec_r);

            tmp = _mm_unpacklo_epi64(_mm_cvtsi64_si128(b), _mm_cvtsi64_si128(g));
            val2 = _mm_cvtsi64_si128(r);
            sh0 = _mm_shuffle_epi8(tmp, vec_mask0);
            sh1 = _mm_shuffle_epi8(val2, vec_mask0);
            // _MM_BLENDV_EPI8(sh0, sh1, vec_mask0_sel)
            val0 = _mm_or_si128(_mm_andnot_si128(vec_mask0_sel, sh0),
                                _mm_and_si128(vec_mask0_sel, sh1));
            // store as 128 bit structure
            _mm_storeu_si128(reinterpret_cast<__m128i *>(dst + width * 3), val0);
            sh0 = _mm_shuffle_epi8(tmp, vec_mask1);
            sh1 = _mm_shuffle_epi8(val2, vec_mask1);
            // _MM_BLENDV_EPI8(sh0, sh1, vec_mask1_sel)
            val2 = _mm_or_si128(_mm_andnot_si128(vec_mask1_sel, sh0),
                                _mm_and_si128(vec_mask1_sel, sh1));
            *(reinterpret_cast<int64_t *>(dst + width * 3 + 16)) = _mm_cvtsi128_si64(val2);

            // second 8 points
            vec_r = _mm_add_epi16(vec_y_high, _mm_mullo_epi16(vec_v_high, vec_param102));
            vec_g = _mm_add_epi16(_mm_add_epi16(vec_y_high,
                                  _mm_mullo_epi16(vec_v_high, vec_param_moin_52)),
                                  _mm_mullo_epi16(vec_u_high, vec_param_moin_25));
            vec_b = _mm_add_epi16(vec_y_high, _mm_mullo_epi16(vec_u_high, vec_param129));

            vec_b = _mm_srai_epi16(vec_b, 6);
            vec_g = _mm_srai_epi16(vec_g, 6);
            vec_r = _mm_srai_epi16(vec_r, 6);

            vec_b = _mm_packus_epi16(vec_b, vec_b);
            vec_g = _mm_packus_epi16(vec_g, vec_g);
            vec_r = _mm_packus_epi16(vec_r, vec_r);

            g = _mm_cvtsi128_si64(vec_g);
            r = _mm_cvtsi128_si64(vec_r);
            b = _mm_cvtsi128_si64(vec_b);
            val2 = _mm_cvtsi64_si128(r);
            tmp = _mm_unpacklo_epi64(_mm_cvtsi64_si128(b), _mm_cvtsi64_si128(g));
            sh0 = _mm_shuffle_epi8(tmp, vec_mask0);
            sh1 = _mm_shuffle_epi8(val2, vec_mask0);
            // _MM_BLENDV_EPI8(sh0, sh1, vec_mask0_sel)
            val0 = _mm_or_si128(_mm_andnot_si128(vec_mask0_sel, sh0),
                                _mm_and_si128(vec_mask0_sel, sh1));
            _mm_storeu_si128(reinterpret_cast<__m128i *>(dst + width * 3 + 24), val0);
            sh0 = _mm_shuffle_epi8(tmp, vec_mask1);
            sh1 = _mm_shuffle_epi8(val2, vec_mask1);
            val2 = _mm_or_si128(_mm_andnot_si128(vec_mask1_sel, sh0),
                                _mm_and_si128(vec_mask1_sel, sh1));
            *(reinterpret_cast<int64_t *>(dst + width * 3 + 40)) = _mm_cvtsi128_si64(val2);

            dst += 48;
            p_y += 16;
        }
        p_y += width;
        dst += 3 * width;
    }

    return true;
}

#else

bool nv12_to_bgr(const unsigned char *src, unsigned char *dst,
                 const int& width, const int& height,
                 const std::string& sensor) {
    cv::Mat yuv(height * 3 / 2, width, CV_8UC1, const_cast<unsigned char *>(src));
    cv::Mat bgr(height, width, CV_8UC3);
    cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);
    memcpy(dst, bgr.data, height * width * 3);
    return true;
}
#endif

}  // namespace airi
}  // namespace crdc
