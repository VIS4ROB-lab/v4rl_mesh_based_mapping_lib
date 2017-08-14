
#ifndef MESH_BASED_MAPPING_FILE_IO_H_
#define MESH_BASED_MAPPING_FILE_IO_H_

namespace mesh_based_mapping {



/*
 *   Beginning of the code adapted from scratchapixel
 *   Copyright (C) 2012  www.scratchapixel.com - GPLv3
 */

float min3(const float &a, const float &b, const float &c) {
  return std::min(a, std::min(b, c));
}

float max3(const float &a, const float &b, const float &c) {
  return std::max(a, std::max(b, c));
}

void maxIndex3(const float &a, const float &b, const float &c, float &maxValue,
               int &maxIndex) {
  if (a > b) {
    if (a > c) {
      maxValue = a;
      maxIndex = 0;
    } else {
      maxValue = c;
      maxIndex = 2;
    }
  } else {
    if (b > c) {
      maxValue = b;
      maxIndex = 1;
    } else {
      maxValue = c;
      maxIndex = 2;
    }
  }
}

float edgeFunction(const cv::Point2f &a, const cv::Point2f &b,
                   const cv::Point2f &c) {
  return (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);
}

void rasterTriangle(const std::vector<cv::Point> &points,
                    const  std::vector<float> &zs,
                    const GEOM_FADE2D::Triangle2 *itri,
                    cv::Mat &resultMap) {

  cv::Point2f v0Raster = points[0];
  cv::Point2f v1Raster = points[1];
  cv::Point2f v2Raster = points[2];
  float x0 = min3(v0Raster.x, v1Raster.x, v2Raster.x);
  float y0 = min3(v0Raster.y, v1Raster.y, v2Raster.y);
  float x1 = max3(v0Raster.x, v1Raster.x, v2Raster.x);
  float y1 = max3(v0Raster.y, v1Raster.y, v2Raster.y);


  float area = edgeFunction(v0Raster, v1Raster, v2Raster);

  float iz0 = 1.0f / zs[0];
  float iz1 = 1.0f / zs[1];
  float iz2 = 1.0f / zs[2];

  for (uint32_t y = y0; y <= y1; ++y) {
    for (uint32_t x = x0; x <= x1; ++x) {
      cv::Point2f pixelSample(x + 0.5f, y + 0.5f);
      float w0 = edgeFunction(v1Raster, v2Raster, pixelSample);
      float w1 = edgeFunction(v2Raster, v0Raster, pixelSample);
      float w2 = edgeFunction(v0Raster, v1Raster, pixelSample);
      w0 /= area;
      w1 /= area;
      w2 /= area;

      if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
        float oneOverZ = iz0 * w0 + iz1 * w1 + iz2 * w2;
        float z = 1.0f / oneOverZ;
        resultMap.at<cv::Vec2f>(y, x)[0] = z;
        int index;
        float maxVal;
        maxIndex3(w0, w1, w2, maxVal, index);
        resultMap.at<cv::Vec2f>(y, x)[1] = itri->getCorner(index)->getCustomIndex();
      }
    }
  }
}

/*
 *   End of the code adapted from scratchapixel
 *   Copyright (C) 2012  www.scratchapixel.com - GPLv3
 */

}


#endif