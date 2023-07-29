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

namespace photoneo
{
  // Pixel depth of B, G, R, and Y channels. Chromatic channels Co and Cg have an additional bit.
  const int PIXEL_DEPTH = 10;

  using ChannelType = std::uint16_t;
  using YCoCgType = ChannelType;
  using RGBType = cv::Vec<std::uint8_t, 3>;

  static RGBType photoneoYCoCgPixelRGB(const ChannelType y, const ChannelType co, const ChannelType cg)
  {
      if (y == ChannelType(0))
          return {0, 0, 0};

      const ChannelType delta = (1 << (PIXEL_DEPTH - 1)); // 2^9
      const ChannelType maxValue = 2 * delta - 1; //2^10 - 1 = 1023
      const ChannelType maxTo8BitDiv = 4; //1023 max, we want max 255

      ChannelType r1 = 2 * y + co;
      ChannelType r = r1 > cg ? (r1 - cg) / 2 : ChannelType(0);
      ChannelType g1 = y + cg / 2;
      ChannelType g = g1 > delta ? (g1 - delta) : ChannelType(0);
      ChannelType b1 = y + 2 * delta;
      ChannelType b2 = (co + cg) / 2;
      ChannelType b = b1 > b2 ? (b1 - b2) : ChannelType(0);

      return {(uint8_t)(std::min(r, maxValue) / maxTo8BitDiv),
              (uint8_t)(std::min(g, maxValue) / maxTo8BitDiv),
              (uint8_t)(std::min(b, maxValue) / maxTo8BitDiv)};
  }

} //namespace photoneo

/**
 * Provides conversion
 *     BGR  <-- YCoCg 4:2:0,
 * where the chromatic channels Co, Cg are subsampled in half resolution.
 * The 2x2 plaquette of the YCoCg format consists of:
 *   (0, 0): Y (10 bits), 1st half of Co (6 bits)
 *   (1, 0): Y (10 bits), 2nd half of Co (6 bits)
 *   (0, 1): Y (10 bits), 1st half of Cg (6 bits)
 *   (1, 1): Y (10 bits), 2nd half of Cg (6 bits)
 *
 * Black pixels are treated specially in order to prevent artifacts in images containing valid pixels only in a subregion.
 *
 * NOTE: The YCoCg format used here differs from the reference by:
 *        - a factor 2 in the both chromatic channels Co, Cg,
 *        - an offset equal to the saturation value, which is added to Co and Cg to prevent negative values.
 *
 * @see https://en.wikipedia.org/wiki/YCoCg
 * @see https://github.com/photoneo-3d/photoneo-cpp-examples/blob/main/GigEV/aravis/common/YCoCg.h
 */

const uint16_t max = 1023;
const uint16_t maxTo8BitShift = 2; //1023 max, we want max 255

inline void rgb_pixel(uint8_t *rgb, const int y, const int csc_co, const int csc_cg)
{
  if(!y)
  {
    *rgb = *(rgb+1) = *(rgb+2) = 0;
    return;
  }

  const int t = y - (csc_cg >> 1);

  const int csc_g = csc_cg + t;
  const int csc_b = t - (csc_co >> 1);
  const int csc_r = csc_co + csc_b;

  const uint8_t r = clamp2(csc_r, 0, (int)max) >> maxTo8BitShift;
  const uint8_t g = clamp2(csc_g, 0, (int)max) >> maxTo8BitShift;
  const uint8_t b = clamp2(csc_b, 0, (int)max) >> maxTo8BitShift;

  *rgb = r;
  *(rgb+1) = g;
  *(rgb+2) = b;
}

void photoneoYCoCg420(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format)
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

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = out->width * sizeof(photoneo::RGBType);
  out->data.resize(out->height * out->step);

  const uint16_t BITS_PER_COMPONENT=10;
  const uint16_t YSHIFT=6;

  const uint16_t maxTo8BitDiv = 4; //1023 max, we want max 255
  const uint16_t maxTo8BitShift = 2; //1023 max, we want max 255

  const size_t rows = in->height;
  const size_t cols = in->width;
  const size_t pixel_offset = 3;
  const size_t rgb_stride = out->step;

  const uint16_t mask = static_cast<uint16_t>((1 << YSHIFT) - 1); //low order 6 bits

  uint16_t *ycocg = (uint16_t*)in->data.data();
  uint8_t *rgb = out->data.data();

  for (size_t row = 0; row < rows; row += 2)
  {
    for (size_t col = 0; col < cols; col += 2)
    {
        const uint16_t y00 = *ycocg >> YSHIFT;
        const uint16_t y01 = *(ycocg+1) >> YSHIFT;

        const uint16_t y10 = *(ycocg+cols) >> YSHIFT;
        const uint16_t y11 = *(ycocg+cols+1) >> YSHIFT;
        // reconstruct Co value from 4:2:0 subsampling
        const uint16_t co = ((*ycocg & mask) << YSHIFT) + (*(ycocg+1) & mask);
        // reconstruct Cg value from 4:2:0 subsampling
        const uint16_t cg = ((*(ycocg+cols) & mask) << YSHIFT) + (*(ycocg+cols + 1) & mask);

        // Note: We employ neareast neighbor interpolation for the subsampled Co, Cg channels.
        //       It's possible to implement bilinear interpolation in those channels.

        const int csc_co = co - (1 << BITS_PER_COMPONENT);
        const int csc_cg = cg - (1 << BITS_PER_COMPONENT);

        rgb_pixel(rgb, y00, csc_co, csc_cg);
        rgb_pixel(rgb+pixel_offset, y01, csc_co, csc_cg);
        rgb_pixel(rgb+rgb_stride, y10, csc_co, csc_cg);
        rgb_pixel(rgb+rgb_stride + pixel_offset, y11, csc_co, csc_cg);

        ycocg += 2;
        rgb += 2*pixel_offset;
    }
    ycocg += cols;
    rgb += rgb_stride;
  }

  out->encoding = out_format;
}

void photoneoYCoCg420_bak(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format)
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

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = out->width * sizeof(photoneo::RGBType);
  out->data.resize(out->height * out->step);

  using namespace photoneo;

  const int yShift = std::numeric_limits<ChannelType>::digits - PIXEL_DEPTH; //16 - 10 = 6
  const ChannelType mask = static_cast<ChannelType>((1 << yShift) - 1); //low order 6 bits

  //wrap around input ROS Image data from buffer pool
  cv::Mat_<YCoCgType> ycocgImg(in->height, in->width, (YCoCgType*)in->data.data(), in->step);

  //wrap around output ROS Image data from buffer pool
  cv::Mat_<RGBType> rgbImg(ycocgImg.rows, ycocgImg.cols, (RGBType*)out->data.data(), out->step);

  for (int row = 0; row < ycocgImg.rows; row += 2)
      for (int col = 0; col < ycocgImg.cols; col += 2)
      {
          const ChannelType y00 = ycocgImg(row, col) >> yShift; //extract Y value (6 bit shift)
          const ChannelType y01 = ycocgImg(row, col + 1) >> yShift;
          const ChannelType y10 = ycocgImg(row + 1, col) >> yShift;
          const ChannelType y11 = ycocgImg(row + 1, col + 1) >> yShift;
          // reconstruct Co value from 4:2:0 subsampling
          const ChannelType co = ((ycocgImg(row, col) & mask) << yShift) + (ycocgImg(row, col + 1) & mask);
          // reconstruct Cg value from 4:2:0 subsampling
          const ChannelType cg = ((ycocgImg(row + 1, col) & mask) << yShift) + (ycocgImg(row + 1, col + 1) & mask);
          // Note: We employ neareast neighbor interpolation for the subsampled Co, Cg channels.
          //       It's possible to implement bilinear interpolation in those channels.
          rgbImg(row, col)         = photoneoYCoCgPixelRGB(y00, co, cg);
          rgbImg(row, col + 1)     = photoneoYCoCgPixelRGB(y01, co, cg);
          rgbImg(row + 1, col)     = photoneoYCoCgPixelRGB(y10, co, cg);
          rgbImg(row + 1, col + 1) = photoneoYCoCgPixelRGB(y11, co, cg);
      }

  out->encoding = out_format;
}

} // end namespace camera_aravis
