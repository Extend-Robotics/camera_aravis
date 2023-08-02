/****************************************************************************
 *
 * camera_aravis
 *
 * Copyright © 2022 Fraunhofer IOSB and contributors
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 ****************************************************************************/

#include <camera_aravis/conversion_utils.h>

#include <ros/ros.h>

#include <opencv2/core/core.hpp> //photoneoMotionCamYCoCg

#include <algorithm> //std::find

namespace camera_aravis
{

void renameImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::renameImg(): no input image given.");
    return;
  }

  // make a shallow copy (in-place operation on input)
  out = in;

  out->encoding = out_format;
}

void shift(uint16_t* data, const size_t length, const size_t digits) {
  for (size_t i=0; i<length; ++i) {
    data[i] <<= digits;
  }
}

void shiftImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const size_t n_digits, const std::string out_format)
{
  if (!in) {
    ROS_WARN("camera_aravis::shiftImg(): no input image given.");
    return;
  }

  // make a shallow copy (in-place operation on input)
  out = in;

  // shift
  shift(reinterpret_cast<uint16_t*>(out->data.data()), out->data.size()/2, n_digits);
  out->encoding = out_format;
}

void interleaveImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const size_t n_digits, const std::string out_format)
{
  if (!in) {
    ROS_WARN("camera_aravis::interleaveImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::interleaveImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = in->step;
  out->data.resize(in->data.size());

  const size_t n_bytes = in->data.size() / (3 * in->width * in->height);
  uint8_t* c0 = in->data.data();
  uint8_t* c1 = in->data.data() + (in->data.size() / 3);
  uint8_t* c2 = in->data.data() + (2 * in->data.size() / 3);
  uint8_t* o = out->data.data();

  for (uint32_t h=0; h<in->height; ++h) {
    for (uint32_t w=0; w<in->width; ++w) {
      for (size_t i=0; i<n_bytes; ++i) {
        o[i] = c0[i];
        o[i+n_bytes] = c1[i];
        o[i+2*n_bytes] = c2[i];
      }
      c0 += n_bytes;
      c1 += n_bytes;
      c2 += n_bytes;
      o += 3*n_bytes;
    }
  }

  if (n_digits>0) {
    shift(reinterpret_cast<uint16_t*>(out->data.data()), out->data.size()/2, n_digits);
  }
  out->encoding = out_format;
}

void unpack10p32Img(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack10pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack10pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (3*in->step)/2;
  out->data.resize((3*in->data.size())/2);

  // change pixel bit alignment from every 3*10+2 = 32 Bit = 4 Byte format LSB
  //  byte 3 | byte 2 | byte 1 | byte 0
  // 00CCCCCC CCCCBBBB BBBBBBAA AAAAAAAA
  // into 3*16 = 48 Bit = 6 Byte format
  //  bytes 5+4       | bytes 3+2       | bytes 1+0
  // CCCCCCCC CC000000 BBBBBBBB BB000000 AAAAAAAA AA000000

  uint8_t* from = in->data.data();
  uint16_t* to = reinterpret_cast<uint16_t*>(out->data.data());
  // unpack a full RGB pixel per iteration
  for (size_t i=0; i<in->data.size()/4; ++i) {

    std::memcpy(to, from, 2);
    to[0] <<= 6;

    std::memcpy(&to[1], &from[1], 2);
    to[1] <<= 4;
    to[1] &= 0b1111111111000000;

    std::memcpy(&to[2], &from[2], 2);
    to[2] <<= 2;
    to[2] &= 0b1111111111000000;

    to+=3;
    from+=4;
  }

  out->encoding = out_format;
}

void unpack10PackedImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack10pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack10pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (3*in->step)/2;
  out->data.resize((3*in->data.size())/2);

  // change pixel bit alignment from every 3*10+2 = 32 Bit = 4 Byte format
  //  byte 3 | byte 2 | byte 1 | byte 0
  // AAAAAAAA BBBBBBBB CCCCCCCC 00CCBBAA
  // into 3*16 = 48 Bit = 6 Byte format
  //  bytes 5+4       | bytes 3+2       | bytes 1+0
  // CCCCCCCC CC000000 BBBBBBBB BB000000 AAAAAAAA AA000000

  // note that in this old style GigE format, byte 0 contains the lsb of C, B as well as A

  uint8_t* from = in->data.data();
  uint8_t* to = out->data.data();
  // unpack a RGB pixel per iteration
  for (size_t i=0; i<in->data.size()/4; ++i) {

    to[0] = from[0]<<6;
    to[1] = from[3];
    to[2] = (from[0] & 0b00001100)<<4;
    to[3] = from[2];
    to[4] = (from[0] & 0b00110000)<<2;
    to[5] = from[1];

    to+=6;
    from+=4;
  }
  out->encoding = out_format;
}


void unpack10pMonoImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack10pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack10pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (8*in->step)/5;
  out->data.resize((8*in->data.size())/5);

  // change pixel bit alignment from every 4*10 = 40 Bit = 5 Byte format LSB
  // byte 4  | byte 3 | byte 2 | byte 1 | byte 0
  // DDDDDDDD DDCCCCCC CCCCBBBB BBBBBBAA AAAAAAAA
  // into 4*16 = 64 Bit = 8 Byte format
  // bytes 7+6        | bytes 5+4       | bytes 3+2       | bytes 1+0
  // DDDDDDDD DD000000 CCCCCCCC CC000000 BBBBBBBB BB000000 AAAAAAAA AA000000

  uint8_t* from = in->data.data();
  uint16_t* to = reinterpret_cast<uint16_t*>(out->data.data());
  // unpack 4 mono pixels per iteration
  for (size_t i=0; i<in->data.size()/5; ++i) {

    std::memcpy(to, from, 2);
    to[0] <<= 6;

    std::memcpy(&to[1], &from[1], 2);
    to[1] <<= 4;
    to[1] &= 0b1111111111000000;

    std::memcpy(&to[2], &from[2], 2);
    to[2] <<= 2;
    to[2] &= 0b1111111111000000;

    std::memcpy(&to[3], &from[3], 2);
    to[3] &= 0b1111111111000000;

    to+=4;
    from+=5;
  }
  out->encoding = out_format;
}

void unpack10PackedMonoImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack10pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack10pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (4*in->step)/3;
  out->data.resize((4*in->data.size())/3);

  // change pixel bit alignment from every 2*10+4 = 24 Bit = 3 Byte format
  //  byte 2 | byte 1 | byte 0
  // BBBBBBBB 00BB00AA AAAAAAAA
  // into 2*16 = 32 Bit = 4 Byte format
  //  bytes 3+2       | bytes 1+0
  // BBBBBBBB BB000000 AAAAAAAA AA000000

  // note that in this old style GigE format, byte 1 contains the lsb of B as well as A

  uint8_t* from = in->data.data();
  uint8_t* to = out->data.data();
  // unpack 4 mono pixels per iteration
  for (size_t i=0; i<in->data.size()/3; ++i) {

    to[0] = from[1]<<6;
    to[1] = from[0];

    to[2] = from[1] & 0b11000000;
    to[3] = from[2];

    to+=4;
    from+=3;
  }
  out->encoding = out_format;
}

void unpack12pImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack12pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack12pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (4*in->step)/3;
  out->data.resize((4*in->data.size())/3);

  // change pixel bit alignment from every 2*12 = 24 Bit = 3 Byte format LSB
  //  byte 2 | byte 1 | byte 0
  // BBBBBBBB BBBBAAAA AAAAAAAA
  // into 2*16 = 32 Bit = 4 Byte format
  //  bytes 3+2       | bytes 1+0
  // BBBBBBBB BBBB0000 AAAAAAAA AAAA0000

  uint8_t* from = in->data.data();
  uint16_t* to = reinterpret_cast<uint16_t*>(out->data.data());
  // unpack 2 values per iteration
  for (size_t i=0; i<in->data.size()/3; ++i) {

    std::memcpy(to, from, 2);
    to[0] <<= 4;

    std::memcpy(&to[1], &from[1], 2);
    to[1] &= 0b1111111111110000;

    to+=2;
    from+=3;
  }
  out->encoding = out_format;
}

void unpack12PackedImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack12pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack12pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (4*in->step)/3;
  out->data.resize((4*in->data.size())/3);

  // change pixel bit alignment from every 2*12 = 24 Bit = 3 Byte format
  //  byte 2 | byte 1 | byte 0
  // BBBBBBBB BBBBAAAA AAAAAAAA
  // into 2*16 = 32 Bit = 4 Byte format
  //  bytes 3+2       | bytes 1+0
  // BBBBBBBB BBBB0000 AAAAAAAA AAAA0000

  // note that in this old style GigE format, byte 1 contains the lsb of B as well as A

  uint8_t* from = in->data.data();
  uint8_t* to = out->data.data();
  // unpack 2 values per iteration
  for (size_t i=0; i<in->data.size()/3; ++i) {

    to[0] = from[1]<<4;
    to[1] = from[0];

    to[2] = from[1] & 0b11110000;
    to[3] = from[2];

    to+=4;
    from+=3;
  }
  out->encoding = out_format;
}

void unpack565pImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack565pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack565pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (3*in->step)/2;
  out->data.resize((3*in->data.size())/2);

  // change pixel bit alignment from every 5+6+5 = 16 Bit = 2 Byte format LSB
  //  byte 1 | byte 0
  // CCCCCBBB BBBAAAAA
  // into 3*8 = 24 Bit = 3 Byte format
  //  byte 2 | byte 1 | byte 0
  // CCCCC000 BBBBBB00 AAAAA000

  uint8_t* from = in->data.data();
  uint8_t* to = out->data.data();
  // unpack a whole RGB pixel per iteration
  for (size_t i=0; i<in->data.size()/2; ++i) {
    to[0] = from[0] << 3;

    to[1] = from[0] >> 3;
    to[1] |= (from[1]<<5);
    to[1] &= 0b11111100;

    to[2] = from[1] & 0b11111000;

    to+=3;
    from+=2;
  }
  out->encoding = out_format;
}

void float_to_uint(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const float scale, const std::string out_format)
{
  const static std::vector<std::string> SUPPORTED_INPUT = {"Coord3D_C32f"}; //GenICam/GiGe-Vision pixel formats

  if (!in)
  {
    ROS_WARN("camera_aravis::float_to_uint(): no input image given.");
    return;
  }

  if (std::find(SUPPORTED_INPUT.begin(), SUPPORTED_INPUT.end(), in->encoding) == SUPPORTED_INPUT.end())
  {
    ROS_WARN("camera_aravis::float_to_uint(): expects float input pixel formats (GenICam/GigE-Vision):");

    for(const std::string &pixel_format : SUPPORTED_INPUT)
      ROS_WARN_STREAM(pixel_format);

    ROS_WARN("if your format is also compatible add it to SUPPORTED_INPUT");
    return;
  }

  if (!out)
  {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::float_to_uint(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = out->width * sizeof(uint16_t);
  out->data.resize(out->height * out->step);

  //wrap around input ROS Image data from buffer pool
  cv::Mat_<float> floatDepth(in->height, in->width, (float*)in->data.data(), in->step);

  //wrap around output ROS Image data from buffer pool
  cv::Mat_<uint16_t> uintDepth(out->height, out->width, (uint16_t*)out->data.data(), out->step);

  floatDepth.convertTo(uintDepth, CV_16UC1, scale);

  uint16_t middlePoint = uintDepth[uintDepth.rows/2][uintDepth.cols/2];

  out->encoding = out_format;
}

/**
 * Provides conversion
 * YCoCg-R 4:2:0 --> BGRA8,
 * where the chromatic channels Co, Cg are subsampled in half resolution.
 * The 2x2 plaquette of the YCoCg format consists of:
 *   (0, 0): Y (10 bits), 1st half of Co (6 bits)
 *   (1, 0): Y (10 bits), 2nd half of Co (6 bits)
 *   (0, 1): Y (10 bits), 1st half of Cg (6 bits)
 *   (1, 1): Y (10 bits), 2nd half of Cg (6 bits)
 *
 * Y is 10 bit
 * Co, Cg are 11 bit
 *
 * Black pixels are protected with y==0 which has to be checked due to 4:2:0 chroma subsampling
 * to prevent artifacts in images containing valid pixels only in subregion.
 *
 * There is no loss of information for other colors
 * as the only RGB that maps to y==0 is black (see color cuboids)
 *
 * Implements YCoCg-R -> RGB conversion as in VESA-DSC-1.2 7.7 Color Space Conversion
 * - convert_rgb = 1
 * - bits_per_component = 10
 *
 * The 10 bit output RGB is scaled to 8 bit RGB
 *
 * Photoneo variantion of YCoCg(-R) doesn't match exactly VESA YCoCg-R
 * but any difference is in LSBs which are lost when scaling to 8 bit RGB.
 *
 * @see https://en.wikipedia.org/wiki/YCoCg
 * @see https://glenwing.github.io/docs/VESA-DSC-1.2.pdf
 * @see https://github.com/photoneo-3d/photoneo-cpp-examples/blob/main/GigEV/aravis/common/YCoCg.h
 * @see https://github.com/photoneo-3d/photoneo-cpp-examples/issues/4
 */

//optimizes to branchless construct
static inline int16_t clamp2(int16_t x, const int16_t min, const int16_t max)
{
  if (x < min) x = min;
  if (x > max) x = max;
  return x;
}

//y positive,  e.g. for 10 bit in [0,1024)
//csc_co and csc_cg centered around zero, e.g. for 10+1 bit in [-1024, 1024)
static inline void ycocgr_to_bgra8(uint8_t *bgra, const int16_t y, const int16_t csc_co, const int16_t csc_cg)
{
  const int16_t MAX_10BIT = 1023;
  const uint8_t MAX_8BIT = 255;
  const uint16_t MAX_10BIT_TO_8BIT_SHIFT = 2; //scale 0-1023 to 0-255 by right shift (division by 4)

  const int16_t t = y - (csc_cg >> 1);

  const int16_t csc_g = csc_cg + t;
  const int16_t csc_b = t - (csc_co >> 1);
  const int16_t csc_r = csc_co + csc_b;

  //The final scaling from 10 bit to 8 bit is deviation from
  //Photoneo sample behavior but we want 8 bit per pixel RGB
  bgra[0] = clamp2(csc_b, 0, MAX_10BIT) >> MAX_10BIT_TO_8BIT_SHIFT;
  bgra[1] = clamp2(csc_g, 0, MAX_10BIT) >> MAX_10BIT_TO_8BIT_SHIFT;
  bgra[2] = clamp2(csc_r, 0, MAX_10BIT) >> MAX_10BIT_TO_8BIT_SHIFT;
  bgra[3] = MAX_8BIT; //alpha channel
}

void photoneoYCoCgR420(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format)
{
  if (!in)
  {
    ROS_WARN("camera_aravis::photoneoMotionCamYCoCg(): no input image given.");
    return;
  }

  if (in->encoding != "Mono16")
  {
    ROS_WARN("camera_aravis::photoneoMotionCamYCoCg(): expects Mono16 encoded custom YCoCg 4:2:0 subsampled data.");
    return;
  }

  if (!out)
  {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::photoneoMotionCamYCoCg(): no output image given. Reserved a new one.");
  }

  const uint16_t BITS_PER_COMPONENT=10; //bit depth of Y while Co and Cg have 1 extra bit
  const uint16_t YSHIFT=6; //10 bits used by Y, 6 bits used by Co/Cg
  const uint16_t COCG_MASK = (uint16_t)((1 << YSHIFT) - 1); //low order 6 bits
  const size_t RGB_PIXEL_OFFSET = 4; //8 bit per channel BGRA (BGR0 compatible)

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = out->width * RGB_PIXEL_OFFSET;
  out->data.resize(out->height * out->step);

  const size_t ROWS = in->height;
  const size_t COLS = in->width;
  const size_t RGB_STRIDE = out->step;

  const uint16_t *ycocg = (uint16_t*)in->data.data();
  uint8_t *bgra = out->data.data();

  for (size_t row = 0; row < ROWS; row += 2)
  {
    for (size_t col = 0; col < COLS; col += 2)
    {
        //readout Y values from 2x2 pixel group
        const uint16_t y00 = ycocg[0] >> YSHIFT;
        const uint16_t y01 = ycocg[1] >> YSHIFT;
        const uint16_t y10 = ycocg[COLS] >> YSHIFT;
        const uint16_t y11 = ycocg[COLS+1] >> YSHIFT;

        // reconstruct Co value from 4:2:0 subsampling
        const uint16_t co = ((ycocg[0] & COCG_MASK) << YSHIFT) | (ycocg[1] & COCG_MASK);
        // reconstruct Cg value from 4:2:0 subsampling
        const uint16_t cg = ((ycocg[COLS] & COCG_MASK) << YSHIFT) | (ycocg[COLS + 1] & COCG_MASK);

        // Co, Cg shared by 2x2 pixel group here but
        // it's possible to implement bilinear interpolation

        // re-center BITS_PER_COMPONENT+1 bit Co and Cg around 0
        // e.g. 11 bit unsingned in [0, 2048)
        // to   11 bit signed    in [-1024, 1024)
        const int16_t csc_co = co - (1 << BITS_PER_COMPONENT);
        const int16_t csc_cg = cg - (1 << BITS_PER_COMPONENT);

        // transfer YCoCg-R to BGRA8

        ////Photoneo specific:
        ////Black pixels are protected with y==0 which has to be checked due to 4:2:0 chroma subsampling
        ////to prevent artifacts in images containing valid pixels only in subregion.
        ////See: https://github.com/photoneo-3d/photoneo-cpp-examples/issues/4#issuecomment-1660578655

        ////Our implementation specific:
        ////(yij != 0) multiplications zero out YCoCb-R if y sample is 0 protecting black pixels
        ////at the same time and keeping high performance SIMD autovectorization
        ////See: https://github.com/Extend-Robotics/camera_aravis/issues/15#issuecomment-1661677749
        ycocgr_to_bgra8(bgra, y00, (y00 != 0) * csc_co, (y00 != 0) * csc_cg);
        ycocgr_to_bgra8(bgra + RGB_PIXEL_OFFSET, y01, (y01 != 0) * csc_co, (y01 != 0) * csc_cg);
        ycocgr_to_bgra8(bgra + RGB_STRIDE, y10, (y10 != 0) * csc_co, (y10 != 0) * csc_cg);
        ycocgr_to_bgra8(bgra + RGB_STRIDE + RGB_PIXEL_OFFSET, y11, (y11 != 0) * csc_co, (y11 != 0) * csc_cg);

        //move to next 2x2 pixel group
        ycocg += 2;
        bgra += 2*RGB_PIXEL_OFFSET;
    }
    //one row was passed in nested loop above
    //move second row for next 2x2 like row
    ycocg += COLS;
    bgra += RGB_STRIDE;
  }

  out->encoding = out_format;
}

} // end namespace camera_aravis
