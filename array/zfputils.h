#ifndef ZFP_UTILS_H
#define ZFP_UTILS_H

#include "zfparray1.h"
#include "zfparray2.h"
#include "zfparray3.h"

namespace zfp {

zfp::array* construct_from_stream(const zfp_header* header, const uchar* buffer, size_t bufferSize = 0)
{
  try {
    return new zfp::array1f(header, buffer, bufferSize);
  } catch (std::exception const & e) {}

  try {
    return new zfp::array1d(header, buffer, bufferSize);
  } catch (std::exception const & e) {}

  try {
    return new zfp::array2f(header, buffer, bufferSize);
  } catch (std::exception const & e) {}

  try {
    return new zfp::array2d(header, buffer, bufferSize);
  } catch (std::exception const & e) {}

  try {
    return new zfp::array3f(header, buffer, bufferSize);
  } catch (std::exception const & e) {}

  try {
    return new zfp::array3d(header, buffer, bufferSize);
  } catch (std::exception const & e) {}

  return NULL;
}

}

#endif
