// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "intrinsic/perception/gpu/cuda_hdr/hdr_mertens.h"

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/gpu/cuda_hdr/hdr_mertens_weight_map.h"
#include "intrinsic/perception/gpu/cuda_image_proc/pixelwise.h"
#include "intrinsic/perception/gpu/cuda_image_proc/pyramid.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_image.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_image_utils.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace perception {
namespace {

absl::Status CheckHdrMertensParams(const HdrMertensParams& params) {
  if (params.contrast_exponent < 0.0f) {
    return absl::InvalidArgumentError("contrast_exponent must be >= 0.0.");
  }
  if (params.saturation_exponent < 0.0f) {
    return absl::InvalidArgumentError("saturation_exponent must be >= 0.0.");
  }
  if (params.well_exposedness_exponent < 0.0f) {
    return absl::InvalidArgumentError(
        "well_exposedness_exponent must be >= 0.0.");
  }
  if (params.pyramid_min_dimensions.cols < 1 ||
      params.pyramid_min_dimensions.rows < 1) {
    return absl::InvalidArgumentError("pyramid_min_dimensions must be >= 1x1.");
  }
  return absl::OkStatus();
}

template <typename InputTraits>
absl::Status ValidateHdrMertensInputs(
    const std::vector<CudaImage<InputTraits>>& ldr_images,
    const HdrMertensParams& params) {
  if (ldr_images.empty()) {
    return absl::InvalidArgumentError("ldr_images must not be empty.");
  }
  INTR_RETURN_IF_ERROR(CheckHdrMertensParams(params));

  const Dimensions base_dims = ldr_images[0].dimensions();
  if (base_dims.cols < params.pyramid_min_dimensions.cols ||
      base_dims.rows < params.pyramid_min_dimensions.rows) {
    return absl::InvalidArgumentError(
        "image dimensions must be >= pyramid_min_dimensions.");
  }
  for (size_t i = 1; i < ldr_images.size(); ++i) {
    if (ldr_images[i].dimensions() != base_dims) {
      return absl::InvalidArgumentError(
          "Image dimensions mismatch. Ensure that all images have the same "
          "dimensions.");
    }
  }
  return absl::OkStatus();
}

}  // namespace

template <typename ImageTraits>
absl::StatusOr<Image<ImageTraits>> ComputeHdrMertens(
    const std::vector<const Image<ImageTraits>*>& ldr_images,
    const HdrMertensParams& params, bool use_float_precision) {
  if (ldr_images.empty()) {
    return absl::InvalidArgumentError("ldr_images must not be empty.");
  }
  INTR_RETURN_IF_ERROR(CheckHdrMertensParams(params));
  std::vector<CudaImage<ImageTraits>> cuda_ldr_images;
  cuda_ldr_images.reserve(ldr_images.size());
  for (const auto& image : ldr_images) {
    INTR_ASSIGN_OR_RETURN(CudaImage<ImageTraits> cuda_image,
                          ToCudaImage(*image));
    cuda_ldr_images.push_back(std::move(cuda_image));
  }

  CudaImage<ImageTraits> cuda_hdr_image;
  if (!use_float_precision) {
    INTR_ASSIGN_OR_RETURN(cuda_hdr_image,
                          ComputeHdrMertens(cuda_ldr_images, params));
  } else {
    using FloatImageTrait = std::conditional<std::is_same_v<ImageTraits, Rgb8u>,
                                             Rgb32f, Gray32f>::type;
    std::vector<CudaImage<FloatImageTrait>> cuda_ldr_images_float;
    cuda_ldr_images_float.reserve(ldr_images.size());
    for (const auto& cuda_ldr_image : cuda_ldr_images) {
      INTR_ASSIGN_OR_RETURN(CudaImage<FloatImageTrait> cuda_ldr_image_float,
                            ConvertImage<FloatImageTrait>(cuda_ldr_image));
      cuda_ldr_images_float.push_back(std::move(cuda_ldr_image_float));
    }
    INTR_ASSIGN_OR_RETURN(const auto cuda_hdr_image_float,
                          ComputeHdrMertens(cuda_ldr_images_float, params));
    INTR_ASSIGN_OR_RETURN(cuda_hdr_image,
                          (ConvertImage<ImageTraits>(cuda_hdr_image_float)));
  }
  INTR_ASSIGN_OR_RETURN(const Image<ImageTraits> host_hdr_image,
                        FromCudaImage(cuda_hdr_image));
  return host_hdr_image;
}

namespace {

template <typename ImageTraits>
struct LaplacianPyramidImageType;

template <>
struct LaplacianPyramidImageType<Gray8u> {
  typedef Generic16i type;
};

template <>
struct LaplacianPyramidImageType<Rgb8u> {
  typedef Generic16i3 type;
};

template <>
struct LaplacianPyramidImageType<Gray32f> {
  typedef Generic32f type;
};

template <>
struct LaplacianPyramidImageType<Rgb32f> {
  typedef Generic32f3 type;
};

}  // namespace

template absl::StatusOr<Image<Gray8u>> ComputeHdrMertens(
    const std::vector<const Image<Gray8u>*>& ldr_images,
    const HdrMertensParams& params, bool use_float_precision);

template absl::StatusOr<Image<Rgb8u>> ComputeHdrMertens(
    const std::vector<const Image<Rgb8u>*>& ldr_images,
    const HdrMertensParams& params, bool use_float_precision);

template <typename ImageTraits>
absl::StatusOr<CudaImage<ImageTraits>> ComputeHdrMertens(
    const std::vector<CudaImage<ImageTraits>>& ldr_images,
    const HdrMertensParams& params) {
  INTR_RETURN_IF_ERROR(ValidateHdrMertensInputs(ldr_images, params));
  std::vector<CudaImage<Generic32f>> weight_maps;
  weight_maps.reserve(ldr_images.size());
  for (const auto& image : ldr_images) {
    INTR_ASSIGN_OR_RETURN(
        auto weight_map,
        ComputeHdrMertensWeightMap(
            image, params.contrast_exponent, params.saturation_exponent,
            params.well_exposedness_exponent, params.border_type));
    weight_maps.push_back(std::move(weight_map));
  }
  INTR_RETURN_IF_ERROR(NormalizeWeightMaps(weight_maps));

  std::vector<CudaImage<GenericTrait<float, ImageTraits::kNumChannels>>>
      blended_pyramid;
  for (int i = 0; i < ldr_images.size(); ++i) {
    INTR_ASSIGN_OR_RETURN(
        const auto gaussian_pyramid_weight_map,
        CreateGaussianPyramid(weight_maps[i], params.pyramid_min_dimensions,
                              params.border_type));

    using LaplacianPyramidImageT = LaplacianPyramidImageType<ImageTraits>::type;
    std::vector<CudaImage<LaplacianPyramidImageT>> laplacian_pyramid_image;

    if constexpr (std::is_same_v<typename ImageTraits::ScalarType, uint8_t>) {
      INTR_ASSIGN_OR_RETURN(laplacian_pyramid_image,
                            CreateLaplacianPyramid<LaplacianPyramidImageT>(
                                ldr_images[i], params.pyramid_min_dimensions,
                                params.border_type));
    } else {
      using FloatImageTrait = GenericTrait<float, ImageTraits::kNumChannels>;
      INTR_ASSIGN_OR_RETURN(const CudaImage<FloatImageTrait> ldr_image_float,
                            Cast<FloatImageTrait>(ldr_images[i]));
      INTR_ASSIGN_OR_RETURN(laplacian_pyramid_image,
                            CreateLaplacianPyramid<LaplacianPyramidImageT>(
                                ldr_image_float, params.pyramid_min_dimensions,
                                params.border_type));
    }
    INTR_RET_CHECK_EQ(gaussian_pyramid_weight_map.size(),
                      laplacian_pyramid_image.size());
    if (blended_pyramid.empty()) {
      blended_pyramid.resize(gaussian_pyramid_weight_map.size());
      for (int j = 0; j < blended_pyramid.size(); j++) {
        INTR_ASSIGN_OR_RETURN(
            blended_pyramid[j],
            (CudaImage<GenericTrait<float, ImageTraits::kNumChannels>>::Create(
                gaussian_pyramid_weight_map[j].dimensions())));
        blended_pyramid[j].SetZero();
      }
    }
    for (int j = 0; j < blended_pyramid.size(); ++j) {
      INTR_RETURN_IF_ERROR(BlendWeightMapAndImage(
          gaussian_pyramid_weight_map[j], laplacian_pyramid_image[j],
          blended_pyramid[j]));
    }
  }
  INTR_ASSIGN_OR_RETURN(CudaImage<ImageTraits> hdr_image,
                        ReconstructImageFromLaplacianPyramid<ImageTraits>(
                            blended_pyramid, params.border_type));
  return hdr_image;
}

template absl::StatusOr<CudaImage<Gray8u>> ComputeHdrMertens(
    const std::vector<CudaImage<Gray8u>>& ldr_images,
    const HdrMertensParams& params);

template absl::StatusOr<CudaImage<Rgb8u>> ComputeHdrMertens(
    const std::vector<CudaImage<Rgb8u>>& ldr_images,
    const HdrMertensParams& params);

template absl::StatusOr<CudaImage<Gray32f>> ComputeHdrMertens(
    const std::vector<CudaImage<Gray32f>>& ldr_images,
    const HdrMertensParams& params);

template absl::StatusOr<CudaImage<Rgb32f>> ComputeHdrMertens(
    const std::vector<CudaImage<Rgb32f>>& ldr_images,
    const HdrMertensParams& params);

}  // namespace perception
}  // namespace intrinsic
